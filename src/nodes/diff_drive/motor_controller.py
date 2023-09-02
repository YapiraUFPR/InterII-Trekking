import rclpy
from geometry_msgs.msg import Twist
from sys import argv
import board
import pwmio
from adafruit_motor import servo
from esc import Esc

ESC_PIN = board.D2
SERVO_PIN = board.D3

servo_motor = None
esc = None

def twist_callback(msg):
    global servo_motor
    global esc

    esc.speed = msg.linear.x
    servo_motor.angle = msg.angular.z

def main():
    rclpy.init(args=argv)

    # node init
    global node
    node = rclpy.create_node('control')
    twist_listener = node.create_subscription(Twist, 'cmd_vel', twist_callback, 10)
    twist_listener
    node.get_logger().info('control node launched.')

    # esc init
    global esc
    esc = Esc(ESC_PIN)

    # servo init
    global servo_motor
    servo_pwm = pwmio.PWMOut(SERVO_PIN, frequency=50)
    servo_motor = servo.Servo(servo_pwm, min_pulse=750, max_pulse=2250)
    
    rclpy.spin(node)

if __name__ == "__main__":
    main()