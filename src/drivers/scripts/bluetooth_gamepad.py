#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import evdev
from time import sleep
import cv2

Y_BUTTON = 308
X_BUTTON = 307
A_BUTTON = 304
B_BUTTON = 305
L1_BUTTON = 310
L2_BUTTON = 312
R1_BUTTON = 311
R2_BUTTON = 313

UP_BUTTON = (17, -1)
DOWN_BUTTON = (17, 1)
UPDOWN_RELEASE = (17, 0)
LEFT_BUTTON = (16, -1)
RIGHT_BUTTON = (16, 1)
LEFTRIGHT_RELEASE = (16, 0)

class BluetoothGamepad(Node):

    def __init__(self):
        super().__init__('bluetooth_gamepad')
        self.logger = self.get_logger()
        self.logger.info('Initializing bluetooth teleop node...')

        # Load config
        fs = cv2.FileStorage("/home/user/ws/src/config/config.yaml", cv2.FileStorage_READ)
        bluetooth_config = fs.getNode("sensors").getNode("bluetooth_gamepad")
        bluetooth_topic = bluetooth_config.getNode("topic").string()
        sample_rate = int(bluetooth_config.getNode("sample_rate").real())
        device_name = bluetooth_config.getNode("device_name").string()
        fs.release()

        # Connect to the controller
        self.gamepad = None
        timeout = 5
        while self.gamepad is None:
            self.logger.info('Connecting to gamepad...')
            try:
                devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
                for device in devices:
                    self.logger.info(f"Found device: {device.name}")
                    if device_name == device.name:
                        self.gamepad = evdev.InputDevice(device.path)
                        break
            except Exception as e:
                self.gamepad = None
                self.logger.error(f"Failed to connect to gamepad: {e}")
                
            if self.gamepad is None:
                self.logger.error(f"Retrying in {timeout} seconds...")
                sleep(timeout)
                timeout *= 2

        self.last_pressed = None

        self.publisher = self.create_publisher(String, bluetooth_topic, 10)
        self.timer = self.create_timer(1/sample_rate, self.timer_callback)

        self.get_logger().info('Bluetooth Publisher Node has been started')

    def timer_callback(self):
        output = "UNKNOWN"
        # Read output from the controller
        for event in self.gamepad.read_loop():

            if event.type == evdev.ecodes.EV_KEY and event.value == 1:
                if event.code == Y_BUTTON:
                    output = "Y"
                elif event.code == X_BUTTON:
                    output = "X"
                elif event.code == A_BUTTON:
                    output = "A"
                elif event.code == B_BUTTON:
                    output = "B"
                elif event.code == L1_BUTTON:
                    output = "L1"
                elif event.code == L2_BUTTON:
                    output = "L2"
                elif event.code == R1_BUTTON:
                    output = "R1"
                elif event.code == R2_BUTTON:
                    output = "R2"
                else:
                    output = "UNKNOWN"
                break
            
            # For d-pad events (absolute axis)
            elif event.type == evdev.ecodes.EV_ABS:
                if (event.code, event.value) == UP_BUTTON:
                    output = "UP_PRESSED"
                    self.last_pressed = "UP"
                elif (event.code, event.value) == DOWN_BUTTON:
                    output = "DOWN_PRESSED"
                    self.last_pressed = "DOWN"
                elif (event.code, event.value) == UPDOWN_RELEASE:
                    if self.last_pressed == "UP":
                        output = "UP_RELEASE"
                    elif self.last_pressed == "DOWN":
                        output = "DOWN_RELEASE"
                elif (event.code, event.value) == LEFT_BUTTON:
                    output = "LEFT_PRESSED"
                    self.last_pressed = "LEFT"
                elif (event.code, event.value) == RIGHT_BUTTON:
                    output = "RIGHT_PRESSED"
                    self.last_pressed = "RIGHT"
                elif (event.code, event.value) == LEFTRIGHT_RELEASE:
                    if self.last_pressed == "LEFT":
                        output = "LEFT_RELEASE"
                    elif self.last_pressed == "RIGHT":
                        output = "RIGHT_RELEASE"
                else:
                    output = "UNKNOWN"
                break
        
        if output != "UNKNOWN":
            msg = String()
            msg.data = output
            self.publisher.publish(msg)
            self.logger.info("Publishing gamepad data...", once=True)
        
if __name__ == "__main__":
    rclpy.init(args=None)

    bt_teleop = BluetoothGamepad()

    rclpy.spin(bt_teleop)
    bt_teleop.destroy_node()
    rclpy.shutdown()
