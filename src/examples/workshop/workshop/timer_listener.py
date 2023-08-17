import rclpy
import rclpy.time
from std_msgs.msg import Header

def listener_callback(msg):
    now = node.get_clock().now()
    difference = now - rclpy.time.Time.from_msg(msg.stamp) 
    seconds, nanoseconds = now.seconds_nanoseconds()
    print(f'Got: "{msg.frame_id}" at {seconds}sec, {nanoseconds}nanosec:')
    print(f'\tIt took {difference.nanoseconds * (10**-9)}sec for the message to arrive.')

def main(args=None):

    rclpy.init(args=args)
    
    global node
    node = rclpy.create_node('minimal_subscriber')
    
    subscription = node.create_subscription(Header, 'topic', listener_callback, 10)
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()