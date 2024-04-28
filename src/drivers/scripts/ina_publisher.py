#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

from time import sleep
from drivers.libs.adafruit_ina219 import INA219
from drivers.libs.i2c import I2C
import cv2
from functools import partial

class BatteryPublisher(Node):

    class Battery:
        def __init__(self, logger, battery_type, cells, capacity, i2c_bus):
            self.battery_type = battery_type
            self.logger = logger

            if battery_type == "LIPO":
                self.cell_voltage = (3.8, 4.2)
            elif battery_type == "LIHV":
                self.cell_voltage = (3.8, 4.35)
            else: 
                self.logger.error(f"Unsupported battery type: {battery_type}")

            self.voltage = (self.cell_voltage[0] * cells, self.cell_voltage[1] * cells)
            self.cells = cells
            self.capacity = capacity
            self.i2c_bus = i2c_bus

            self.delta_voltage = self.voltage[1] - self.voltage[0]

            ina219 = None
            timeout = 5
            while ina219 is None:
                self.logger.info('Initializing sensor INA219...')
                try:
                    i2c = I2C(self.i2c_bus, 5*1000)
                    ina219 = INA219(i2c)
                    ina219.set_calibration_16V_5A()
                    self.sensor = ina219
                except Exception as e:
                    ina219 = None
                    self.logger.error(f"Failed to initialize INA219 at bus {self.i2c_bus}: {e}")
                    self.logger.error(f"Retrying in {timeout} seconds...")
                    sleep(timeout)
                    timeout *= 2

    def __init__(self):
        super().__init__('ina_publisher')
        self.logger = self.get_logger()
        self.logger.info('Initializing battery sensor node...')

        # load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        battery_config = fs.getNode("sensors").getNode("battery")
        sample_rate = int(battery_config.getNode("sample_rate").real())
        sensors_num = battery_config.getNode("topic").size()
        self.sensors_num = sensors_num
        topics = [battery_config.getNode("topic").at(i).string() for i in range(sensors_num)]
        battery_types = [battery_config.getNode("type").at(i).string() for i in range(sensors_num)]
        cells = [int(battery_config.getNode("cells").at(i).real()) for i in range(sensors_num)]
        capacities = [battery_config.getNode("capacity").at(i).real() for i in range(sensors_num)]
        i2c_buses = [int(battery_config.getNode("bus").at(i).real()) for i in range(sensors_num)]
        fs.release()

        # init battery sensors
        self.batteries = [self.Battery(self.logger, battery_types[i], cells[i], capacities[i], i2c_buses[i]) for i in range(sensors_num)]

        # init publishers
        self.bat_publishers = []
        self.bat_timers = []
        for i in range(sensors_num):
            self.bat_publishers.append(self.create_publisher(BatteryState, topics[i], 10))
            
        self.bat_timer = self.create_timer(1/sample_rate, self.timer_callback)

        self.logger.info('Battery sensor node launched.')

    def timer_callback(self):
        self.logger.info("Publishing battery data...", once=True)

        for idx in range(self.sensors_num):
            battery = self.batteries[idx]
            
            bus_voltage = battery.sensor.bus_voltage
            # shunt_voltage = ina219.shunt_voltage
            current = battery.sensor.current
            # power = ina219.power
            percentage = (bus_voltage - battery.voltage[0]) / battery.delta_voltage

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = bus_voltage
            msg.current = current
            msg.charge = percentage * battery.capacity
            msg.capacity = battery.capacity
            msg.design_capacity = battery.capacity
            msg.percentage = percentage

            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            
            if battery.battery_type == "LIPO":
                msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            elif battery.battery_type == "LIHV":
                msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            else:
                msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

            msg.present = True

            self.bat_publishers[idx].publish(msg)

if __name__ == "__main__":
    rclpy.init(args=None)

    battery_publisher = BatteryPublisher()

    rclpy.spin(battery_publisher)
    battery_publisher.destroy_node()
    rclpy.shutdown()