import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

TOPIC = "/fake_pose"
FREQUENCY = 30

class FakePosePublisher(Node):

    def __init__(self):
        super().__init__("fake_pose_publisher")

        self.logger = self.get_logger()
        self.logger.info("Starting fake pose publisher...")
        
        self.curr_pose = np.array([0.0, 0.0, 0.0]) # in 3D

        self.publisher = self.create_publisher(PoseStamped, "/fake_pose", 10)
        self.timer = self.create_timer(1/FREQUENCY, self.timer_callback)

        self.logger.info("Fake pose publisher started.")

    def timer_callback(self):

        self.logger.info("Publishing fake pose...", once=True)

        # Select a random direction to move and a random distance.
        distance = np.random.uniform(0.0, 0.1)
        angle = np.random.uniform(-np.pi/6, np.pi/6)

        # Update the current pose.
        self.curr_pose[0] += distance * np.cos(self.curr_pose[2])
        self.curr_pose[1] += distance * np.sin(self.curr_pose[2])
        self.curr_pose[2] += angle

        # Prepare and publish ROS message.
        msg = PoseStamped()

        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = self.curr_pose[0]
        msg.pose.position.y = self.curr_pose[1]
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = np.sin(self.curr_pose[2]/2)

        self.publisher.publish(msg)


if __name__ == "__main__":
    rclpy.init(args=None)

    fake_pose_publisher = FakePosePublisher()
    rclpy.spin(fake_pose_publisher)

    fake_pose_publisher.destroy_node()
    rclpy.shutdown()