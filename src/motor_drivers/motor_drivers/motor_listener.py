import rclpy
from geometry_msgs.msg import Twist
from gpiozero import Servo
import yaml
from time  import sleep

taget_speed = 0
target_angle = 0

def twist_callback(msg):
    global servo_motor
    global esc

    angle = msg.angular.z
    angle = max(0, min(angle, 1))

    speed = msg.linear.x
    speed = max(-1, min(speed, 1))

    print(speed, angle)

    global taget_speed
    global target_angle
    taget_speed = speed
    target_angle = angle


def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["motors"]["node"]
    topic = config["motors"]["topic"]
    sample_rate = config["motors"]["sample_rate"]
    esc_pin = config["motors"]["escpin"]
    servo_pin = config["motors"]["servo_pin"]

    # node init
    rclpy.init()
    global node
    node = rclpy.create_node(node_name)
    twist_listener = node.create_subscription(Twist, topic, twist_callback, 10)
    rate = node.create_rate(sample_rate)  # frequency in Hz
    twist_listener, rate
    logger = node.get_logger()
    logger.info('Motors node launched.')

    # motors init
    esc = Servo(esc_pin, initial_value=-1.0)
    servo_motor = Servo(servo_pin, initial_value=0)
    sleep(3)
    esc.value = 0.0
    sleep(5)
    logger.info("Esc initialized.") 


    global taget_speed
    global target_angle

    while True:
        speed_inc = 0.01 if taget_speed > 0 else -0.01
        speed = esc.value + speed_inc
        esc.value = max(-1, min(speed, 1))

        angle_inc = 0.01 if target_angle > 90 else -0.01
        angle = servo_motor.value + angle_inc
        servo_motor.value = max(-1, min(angle, 1))

        rclpy.spin_once(node)

if __name__ == "__main__":
    main()
