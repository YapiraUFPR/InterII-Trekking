import rclpy
import yaml
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from monocular_visual_odometry import MonoVideoOdometery

image_buffer = []

def camera_callback(msg):
    global node
    global image_buffer
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_buffer.append(image_np)

def odometry():
    
    # load config
    with open("/home/user/ws/src/config/config.yaml", "r") as file:
        config = yaml.safe_load(file)
    camera_topic = config["sensors"]["camera"]["topic"]

    # ros2 initialization
    rclpy.init(args=None)
    global node
    node = rclpy.create_node("camera_listener")
    camera_sub = node.create_subscription(CompressedImage, camera_topic, camera_callback, 10)
    optflow_pub = node.create_publisher(CompressedImage, "/optflow", 10)
    
    rate = node.create_rate(10) # frequency in Hz
    camera_sub, rate
    logger = node.get_logger()
    logger.info('Camera listener node launched.')


    # get first frame
    global image_buffer
    while len(image_buffer) == 0:
        rclpy.spin_once(node)

    # map output
    colors = np.random.randint(0,255,(5000,3))
    traj = np.zeros(shape=(600, 800, 3))

    first_frame = image_buffer.pop(0)
    gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    vo = MonoVideoOdometery(gray)

    while True:
        rclpy.spin_once(node)

        if len(image_buffer) > 0:
            curr_frame = image_buffer.pop(0)
            
            gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
            vo.process_frame(gray)
            pose = vo.get_mono_coordinates()

            print("x: {}, y: {}, z: {}".format(*[str(pt) for pt in pose]))

            draw_x, draw_y, draw_z = [int(round(x)) for x in pose]

            traj = cv2.circle(traj, (draw_x + 400, draw_z + 100), 1, list((0, 255, 0)), 4)

            cv2.putText(traj, 'Actual Position:', (140, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
            cv2.putText(traj, 'Red', (270, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 255), 1)
            cv2.putText(traj, 'Estimated Odometry Position:', (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255), 1)
            cv2.putText(traj, 'Green', (270, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0), 1)

            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', traj)[1]).tobytes()

            optflow_pub.publish(msg)

if __name__ == "__main__":
    odometry()