import rclpy
from std_msgs.msg import String

def listener_callback(msg):
    print(f'Got: "{msg.data}"')

def main(args=None):

    rclpy.init(args=args)
    
    global node
    node = rclpy.create_node('minimal_subscriber')
    
    
    subscription = node.create_subscription(String, 'topic', listener_callback, 10)
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()