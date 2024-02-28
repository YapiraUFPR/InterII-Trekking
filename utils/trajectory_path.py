import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sys import argv

path = Path()

def pose_callback(msg):
    global path
    path.header = msg.header
    path.poses.append(msg)

def main():

    rclpy.init(args=None)
    node = rclpy.create_node('trajectory_path')
    pose_sub = node.create_subscription(PoseStamped, argv[1], pose_callback, 10)
    path_pub = node.create_publisher(Path, argv[2], 10)
    pose_sub

    while True:
        rclpy.spin_once(node)

        if len(path.poses) > 0:
            path_pub.publish(path)

if __name__ == '__main__':
    main()
