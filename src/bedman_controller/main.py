import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import yaml
from time import sleep
from pid import PID
from time import time_ns
import numpy as np

class Controller:
    IDLE = 0
    PID_STATE = 1
    CORRECTING = 2

    LINEAR_SPEED = 0.25

    def __init__(self):
        # load config
        with open("/home/user/ws/src/config/config.yaml", "r") as file:
            config = yaml.safe_load(file)
            self.mark_topic = config["led"]["topic"]
            self.pid_topic = config["twod_pid"]["topic"]
            self.motors_topic = config["motors"]["topic"]
            self.imu_topic = config["sensors"]["imu"]["topic"]

        # ros2 initialization
        rclpy.init()
        self.node = rclpy.create_node("main_controller")
        self.mark_sub = self.node.create_subscription(Bool, self.mark_topic, self.mark_callback, 10)
        self.imu_sub = self.node.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.pid_correction = self.node.create_subscription(Twist, self.pid_topic, self.pid_callback, 10)
        self.speed_pub = self.node.create_publisher(Twist, self.motors_topic, 10)

        self.logger = self.node.get_logger()

        self.state = self.IDLE 
        self.logger.info("Control node initialized")
        self.in_mark = False

        self.angle_buffer = []
        self.angular_velocity = 0.0

    def imu_callback(self, msg:Imu):
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        
        if angular_velocity[2] > 0.1:
            self.angle_buffer.append(angular_velocity[2])

    def mark_callback(self, msg:Bool):
        self.in_mark = msg.data

    def pid_callback(self, msg:Twist):
        if self.state == self.PID_STATE:
            self.speed_pub.publish(msg)

    def set_speed(self, speed, angle):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = angle
        self.speed_pub.publish(twist_msg)

    def straight(self, duration):
        # use imu data to go in a straight line 
        self.set_speed(self.LINEAR_SPEED, 0.0)
        pid = PID(0.15, 0.01, 0.0, setpoint=0.0)
        pid.output_limits = (-1.0, 1.0)
        start_time = time_ns() // 1e6
        while ((time_ns() / 1e6) - start_time) < duration:
            
            if len(self.angle_buffer) > 0:
                angle = np.mean(self.angle_buffer)
                self.angle_buffer = []

                self.logger.info(f"Angle: {angle}")
                
                angle_correction = pid(angle)
                
                self.logger.info(f"Angle correction: {angle_correction}")

                self.set_speed(self.LINEAR_SPEED, angle_correction)
            
            rclpy.spin_once(self.node)

    def run(self):
        # get to the first mark
        self.logger.info("Will try to go straight for 1000 ms")
        self.state = self.CORRECTING
        self.straight(7000)
        self.state = self.PID_STATE

        while not self.in_mark:
            self.logger.info("Correcting in mark direction")
            rclpy.spin_once(self.node)
        self.set_speed(0.0, 0.0)

        # self.state = self.PID_STATE
        # while not self.in_mark:
        #     self.logger.info("Correcting in mark direction")
        #     rclpy.spin_once(self.node)
        # self.set_speed(0.0, 0.0)
        # self.logger.info("Reached first mark")
        # self.state = self.CORRECTING
        # sleep(1)

        # self.logger.info("Backing...")
        # self.set_speed(-self.LINEAR_SPEED, 0.0)
        # sleep(1.5)
        # self.logger.info("Aligning...")
        # self.set_speed(self.LINEAR_SPEED, 0.5)
        # sleep(1.0)
        # self.set_speed(self.LINEAR_SPEED, 5.0)
        # sleep(0.5)
        # self.logger.info("Will enter PID mode again")
        # self.set_speed(0.0, 0.0)

        # self.state = self.PID_STATE
        # while not self.in_mark:
        #     self.logger.info("Correcting in mark direction")
        #     rclpy.spin_once(self.node)
        # self.set_speed(0.0, 0.0)
        # self.logger.info("Reached second mark")

if __name__ == "__main__":
    controller = Controller()
    controller.run()