import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from time import sleep

global marker_pub

def pose_callback(msg):
    maker_msg = Marker()
    maker_msg.header.stamp = msg.header.stamp
    maker_msg.header.frame_id = msg.header.frame_id
    maker_msg.pose = msg.pose
    maker_msg.type = Marker.SPHERE
    maker_msg.action = Marker.ADD
    maker_msg.scale.x = 1.0
    maker_msg.scale.y = 1.0
    maker_msg.scale.z = 1.0
    maker_msg.color.a = 1.0
    maker_msg.color.r = 1.0
    maker_msg.color.g = 0.0
    maker_msg.color.b = 0.0
    maker_msg.lifetime.sec = 1000

    marker_pub.publish(maker_msg)

def main():
    global marker_pub

    rclpy.init()
    marker_node = rclpy.create_node('marker_node')
    marker_pub = marker_node.create_publisher(Marker, '/dummy_marker', 10)
    pose_sub = marker_node.create_subscription(PoseStamped, '/dummy_pose', pose_callback, 10)
    rclpy.spin(marker_node)
    


if __name__ == "__main__":
    main()