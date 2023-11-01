import rclpy
import yaml
from sensor_msgs.msg import BatteryState
from sys import argv
from adafruit_ina219 import INA219
import adafruit_bitbangio as bitbangio
import board

def battery_node():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["battery"]["node"]
    topic = config["battery"]["topic"]
    sample_rate = config["battery"]["sample_rate"]
    max_cell_voltage = config["battery"]["max_cell_voltage"]
    min_cell_voltage = config["battery"]["min_cell_voltage"]
    cells = config["battery"]["cells"]
    capacity = config["battery"]["capacity"]

    max_voltage = max_cell_voltage * cells
    min_voltage = min_cell_voltage * cells

    # ros2 initialization
    rclpy.init(args=argv)
    global node
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(BatteryState, topic, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    logger = node.get_logger()
    logger.info('Battery sensor node launched.')

    # sensor initialization
    i2c = bitbangio.I2C(scl=board.D5, sda=board.D6, frequency=sample_rate*1000)
    ina219 = INA219(i2c)
    ina219.set_calibration_16V_5A()

    # main loop
    logger.info('Publishing battery data...')
    while True:
        msg = BatteryState()
        msg.header.stamp = node.get_clock().now().to_msg()

        bus_voltage = ina219.bus_voltage
        # shunt_voltage = ina219.shunt_voltage
        current = ina219.current
        # power = ina219.power
        percentage = (bus_voltage - min_voltage) / (max_voltage - min_voltage)

        msg.voltage = bus_voltage
        msg.current = current
        msg.charge = percentage * capacity
        msg.capacity = capacity
        msg.design_capacity = capacity
        msg.percentage = percentage

        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        msg.present = True

        pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    battery_node()