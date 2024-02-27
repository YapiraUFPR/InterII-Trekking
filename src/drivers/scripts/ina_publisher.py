#!/usr/bin/env python3
import rclpy
import yaml
from sensor_msgs.msg import BatteryState
from time import sleep
from .libs.adafruit_ina219 import INA219
from .libs.i2c import I2C
import board

class Battery:
    def __init__(self, config, sample_rate):
        self.sample_rate = sample_rate
        self.topic = config["topic"]
        self.max_cell_voltage = config["max_cell_voltage"]
        self.min_cell_voltage = config["min_cell_voltage"]
        self.cells = config["cells"]
        self.capacity = config["capacity"]
        self.i2c_bus = config["bus"]

        self.min_voltage = self.min_cell_voltage * self.cells
        self.max_voltage = self.max_cell_voltage * self.cells
        self.delta_voltage = self.max_voltage - self.min_voltage

        self.pub = None
        self.sensor = None

def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)

    battery_configs = config["sensors"]["battery"]["sensors"]
    sample_rate = battery_configs["sensors"]["battery"]["sample_rate"]

    batteries = [Battery]
    for battery_config in battery_configs:
        battery = Battery(battery_config, sample_rate)
        batteries.append(battery)

    # ros2 initialization
    rclpy.init()
    global node
    node = rclpy.create_node("ina219")
    logger = node.get_logger()
    for i in range(len(batteries)):
        pub = node.create_publisher(BatteryState, batteries[i].topic, 10)
        batteries[i].pub = pub
    logger.info('Battery sensor node launched.')


    # sensor initialization
    for i in range(len(batteries)):
        ina219 = None
        timeout = 5
        while ina219 is None:
            logger.info('Initializing sensor INA219...')
            try:
                i2c = I2C(batteries[i].i2c_bus, sample_rate*1000)
                ina219 = INA219(i2c)
                ina219.set_calibration_16V_5A()
                batteries[i].sensor = ina219
            except Exception as e:
                ina219 = None
                logger.error(f"Failed to initialize INA219: {e}")
                logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2

    # main loop
    logger.info('Publishing battery data...')
    while rclpy.ok():
        for battery in batteries:
            msg = BatteryState()
            msg.header.stamp = node.get_clock().now().to_msg()

            bus_voltage = battery.sensor.bus_voltage
            # shunt_voltage = ina219.shunt_voltage
            current = battery.sensor.current
            # power = ina219.power
            percentage = (bus_voltage - battery.min_voltage) / (battery.max_voltage - battery.min_voltage)

            msg.voltage = bus_voltage
            msg.current = current
            msg.charge = percentage * battery.capacity
            msg.capacity = battery.capacity
            msg.design_capacity = battery.capacity
            msg.percentage = percentage

            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

            msg.present = True

            battery.pub.publish(msg)

        sleep(1/sample_rate)

if __name__ == "__main__":
    main()