import rclpy
from geometry_msgs.msg import Twist
from gpiozero import Servo
import yaml
from time  import sleep
import subprocess

# run sudo pigpiod
try:
    subprocess.Popen(["sudo", "pigpiod"])
except Exception:
    pass 

target_speed = 0.0
target_angle = 0.0

def twist_callback(msg):
    global node
    global target_speed
    global target_angle

    logger = node.get_logger()

    speed = msg.linear.x
    angle = msg.angular.z
    logger.info(f"Received twist msg with {speed} and {angle}")

    speed = max(-1, min(speed, 1))
    angle = max(-1, min(angle, 1))

    target_speed = speed
    target_angle = angle
    logger.info(f"Speed: {speed}, angle: {angle}")


def main():

    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    node_name = config["motors"]["node"]
    topic = config["motors"]["topic"]
    sample_rate = config["motors"]["sample_rate"]
    esc_pin = config["motors"]["esc_pin"]
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
    esc = Servo(esc_pin, initial_value=0.0)
    servo_motor = Servo(servo_pin, initial_value=0)
    sleep(3)
    esc.value = 0.0
    sleep(5)
    logger.info("Esc initialized.") 


    global target_speed
    global target_angle

    speed = 0.0
    angle = 0.0
    try:
        while True:
            speed_diff = target_speed - speed
            if speed_diff != 0.0:
                speed_inc = 0.01 if speed_diff > 0.0 else -0.05
                speed = esc.value + speed_inc
                esc.value = max(-1, min(speed, 1))

            ang_diff = target_angle - angle
            if ang_diff != 0.0:
                angle_inc = 0.01 if ang_diff > 0.0 else -0.05
                angle = servo_motor.value + angle_inc
                servo_motor.value = max(-1, min(angle, 1))

            # esc.value = target_speed
            # servo_motor.value = target_angle

            logger.info(f"Current speed: {speed}, {angle}")
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        esc.value = 0.0
        servo_motor.value = 0.0
        logger.info("Motors stopped.") 
        return

if __name__ == "__main__":
    main()
