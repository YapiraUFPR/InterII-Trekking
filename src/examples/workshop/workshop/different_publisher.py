import rclpy
from std_msgs.msg import String

def timer_callback():
    msg = String()
    msg.data = INPUT
    publisher.publish(msg)
    print(f'Sent:"{msg.data}"')

def main(args=None):

    rclpy.init(args=args)

    global node
    node = rclpy.create_node('minimal_publisher')

    global publisher
    publisher = node.create_publisher(String, 'topic', 10)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)
    timer

    global INPUT
    INPUT = ''

    while rclpy.ok():
        INPUT = input()
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
