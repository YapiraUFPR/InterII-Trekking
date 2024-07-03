import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import bluetooth
import subprocess

class BluetoothPublisher(Node):

    def __init__(self):
        super().__init__('bluetooth_publisher')
        self.publisher_ = self.create_publisher(String, 'bluetooth_devices', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Bluetooth Publisher Node has been started')

    def timer_callback(self):
        self.publish_connected_devices()

    def publish_connected_devices(self):
        result = subprocess.run(['hcitool', 'con'], stdout=subprocess.PIPE)
        output = result.stdout.decode('utf-8')
        
        msg = String()
        msg.data = "Connected devices:\n" + output
        self.publisher_.publish(msg)
        self.get_logger().info('Published connected devices.')

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothPublisher()

    rclpy.spin(node)

if __name__ == "__main__":
    main()
