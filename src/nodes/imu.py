import rclpy
from sensor_msgs.msg import Imu

def main(args=None):

    global scan
    scan = []

    rclpy.init(args=args)

    global node
    node = rclpy.create_node('imu')

    global publisher
    publisher = node.create_publisher(Imu, 'imu_reading', rclpy.qos.qos_profile_sensor_data)

    timer = node.create_timer(0.5, timer_callback)
    timer

    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()