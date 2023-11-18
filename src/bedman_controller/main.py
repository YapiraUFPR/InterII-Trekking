import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import yaml
from time import sleep

class Controller:
    IDLE = 0
    PID_STATE = 1
    CORRECTING = 2

    LINEAR_SPEED = 0.2

    def __init__(self):
        # load config
        with open("/home/user/ws/src/config/config.yaml", "r") as file:
            config = yaml.safe_load(file)
            self.mark_topic = config["led"]["topic"]
            self.pid_topic = config["pid"]["topic"]
            self.motors_topic = config["motors"]["topic"]

        # ros2 initialization
        rclpy.init()
        self.node = rclpy.create_node("main_controller")
        self.mark_sub = self.node.create_subscription(Bool, self.mark_topic, self.mark_callback, 10)
        self.pid_correction = self.node.create_subscriber(Twist, self.pid_topic, self.pid_callback, 10)
        self.speed_pub = self.node.create_publisher(Twist, self.motors_topic, 10)

        self.logger = self.node.get_logger()

        self.state = self.IDLE 
        self.logger.info("Control node initialized")

    def mark_callback(self, msg:Bool):
        self.in_mark = msg.data

    def pid_callback(self, msg:Twist):
        if self.state == self.PID_STATE:
            self.speed_pub.publish(msg)

    def set_speed(speed, angle):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = angle

    def run(self):
        # get to the first mark
        self.state = self.CORRECTING
        self.set_speed(0.2, 0.0)
        sleep(1.5)
        
        self.state = self.PID_STATE
        while not self.in_mark:
            self.logger.info("Correcting in mark direction")
            rclpy.spin_once(self.node)
        self.set_speed(0.0, 0.0)
        self.logger.info("Reached first mark")
        self.state = self.CORRECTING
        sleep(1)

        self.logger.info("Backing...")
        self.set_speed(-self.LINEAR_SPEED, 0.0)
        sleep(1.5)
        self.logger.info("Aligning...")
        self.set_speed(self.LINEAR_SPEED, 0.5)
        sleep(1.0)
        self.set_speed(self.LINEAR_SPEED, 5.0)
        sleep(0.5)
        self.logger.info("Will enter PID mode again")
        self.set_speed(0.0, 0.0)

        self.state = self.PID_STATE
        while not self.in_mark:
            self.logger.info("Correcting in mark direction")
            rclpy.spin_once(self.node)
        self.set_speed(0.0, 0.0)
        self.logger.info("Reached second mark")

if __name__ == "__main__":
    controller = Controller()
    controller.run()