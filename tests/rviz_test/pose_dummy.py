import rclpy
from geometry_msgs.msg import PoseStamped
from time import sleep

def main():

    rclpy.init()
    pose_node = rclpy.create_node('pose_node')
    pose_pub = pose_node.create_publisher(PoseStamped, 'dummy_pose', 10)

    msg = PoseStamped()
    x, y, z = 0.0, 0.0, 0.0
    while True:

        msg.header.stamp = pose_node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        pose_pub.publish(msg)

        if z < 10:
            z += 1
        else:
            x += 1

        sleep(1)


if __name__ == "__main__":
    main()