import rclpy
from geometry_msgs.msg import PoseStamped
from sys import argv

def pose_callback(msg):
    with open(argv[1], 'a') as file:
        file.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec},{msg.pose.position.x},{msg.pose.position.y},{msg.pose.position.z}\n")

def main():
    rclpy.init(args=None)
    node = rclpy.create_node('pose_logger')
    pose_sub = node.create_subscription(PoseStamped, argv[2], pose_callback, 10)

    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == '__main__':
    if len(argv) != 3:
        print("Usage: python pose_logger.py <output_file> <pose_topic>")
    else:
        main()
