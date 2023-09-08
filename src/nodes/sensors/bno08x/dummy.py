import rclpy
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
import time
import sys
from time import sleep

FILE_PATH = "/home/gab/projetos/yapira/bedman-trekker/src/nodes/sensors/bno08x/data.txt"

def bno08x_node():
    rclpy.init(args=sys.argv)

    global node
    node = rclpy.create_node('bno08x')

    raw_pub = node.create_publisher(Imu, 'bno08x/raw', 10)
    mag_pub = node.create_publisher(MagneticField, 'bno08x/mag', 10)
    # status_pub = node.create_publisher(DiagnosticStatus, 'bno08x/status', 10)
    rate = node.create_rate(100)  # frequency in Hz
    node.get_logger().info('bno08x node launched.')
    mag_pub, rate

    # load data from file
    data = []
    with open(FILE_PATH, 'r') as f:
        for line in f:
            print(line)
            line_data = [float(x) for x in (line.split(' '))]

    data = [[float(x) for x in line] for line in data]

    i = 0
    while rclpy.ok():
        raw_msg = Imu()
        raw_msg.header.stamp = node.get_clock().now().to_msg()

        accel_x, accel_y, accel_z = data[i][4:7]
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        gyro_x, gyro_y, gyro_z = data[i][1:4]
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        quat_i, quat_j, quat_k, quat_real = data[i][10:14]
        raw_msg.orientation.w = quat_i
        raw_msg.orientation.x = quat_j
        raw_msg.orientation.y = quat_k
        raw_msg.orientation.z = quat_real

        raw_pub.publish(raw_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = node.get_clock().now().to_msg()

        mag_x, mag_y, mag_z = data[i][7:10]
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        mag_pub.publish(mag_msg)
        raw_pub.publish(raw_msg)

        i += 1

        rate.sleep()

    node.destroy_node()

if __name__ == '__main__':
    bno08x_node()