import rclpy
from std_msgs.msg import Header

def send_message(INPUT):
    msg = Header()
    msg.frame_id = INPUT
    msg.stamp = node.get_clock().now().to_msg()
    publisher.publish(msg)
    print(f'Sent:"{msg.frame_id}" at {msg.stamp.sec}sec, {msg.stamp.nanosec}nanosec')

def main(args=None):

    rclpy.init(args=args)

    global node
    node = rclpy.create_node('minimal_publisher')

    global publisher
    publisher = node.create_publisher(Header, 'topic', 10)

    while rclpy.ok():
        INPUT = input()
        send_message(INPUT)
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()