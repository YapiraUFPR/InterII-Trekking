import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global scan
    scan = msg.ranges

SPEED = 0.3

def timer_callback():
    # print(f'0*: {scan[0]}, 90*: {scan[90]}, 180*: {scan[180]}, 270*: {scan[270]}')
    msg = Twist()

    arc = scan[:50] + scan[310:]
    distance_ahead = min(arc)

    print(distance_ahead)

    if distance_ahead < 0.2:
        msg.linear.x = -SPEED

    elif distance_ahead < 0.4:
        msg.angular.z = SPEED

    else:
        msg.linear.x = SPEED

    publisher.publish(msg)


def main(args=None):

    global scan
    scan = []

    rclpy.init(args=args)

    global node
    node = rclpy.create_node('wanderbot')

    global publisher
    publisher = node.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_system_default)
    sub = node.create_subscription(LaserScan, 'scan', scan_callback, rclpy.qos.qos_profile_sensor_data)
    sub

    timer = node.create_timer(0.5, timer_callback)
    timer

    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()