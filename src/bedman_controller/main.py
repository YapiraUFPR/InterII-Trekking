import rclpy
from std_msgs.msg import Bool
import yaml

class Controller:
    def __init__(self):
        # load config
        with open("/home/user/ws/src/config/config.yaml", "r") as file:
            config = yaml.safe_load(file)
            self.mark_topic = config["led"]["topic"]
        
        # ros2 initialization
        rclpy.init()
        self.node = rclpy.create_node("state_machine")
        self.mark_sub = self.node.create_subscription(Bool, self.mark_topic, self.mark_callback, 10)
        
        self.state = "idle"

    def mark_callback(self, msg:Bool):
        if msg.data:
            self.state = "mark"
        else:
            self.state = "idle"

    def run(self):
        # get to the first mark
        while self.state != "mark":
