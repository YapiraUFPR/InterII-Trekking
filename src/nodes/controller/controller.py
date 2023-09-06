import rclpy
import numpy as np 
from geometry_msgs.msg import PoseStamped, Twist
from sys import argv

MAP_PATH = "../carlos/map.npy"
map = np.load(MAP_PATH)

velocity = 0
PATH_RADIUS = 10

LINEAR_SPEED = 30

def position_callback(msg):
    global node 
    global map

    x, y, z = msg.pose.position 

    # predict future position 2 seconds from now
    future_pos = np.array([x, y]) + velocity * 2

    normal, target = None
    closest_dist = np.inf
    for i in range(len(map)):
        
        # line segment in map
        a = map[i]
        b = map[(i+1)%len(map)] 

        # get normal vector between line segment and future position
        ap = future_pos - a
        ab = b - a
        ab = ab / np.linalg.norm(ab)
        normal_point = ab * np.dot(ap, ab)
        normal_point += a

        direction = b - a

        # check if normal point is on line segment
        if np.linalg.norm(normal_point - a) + np.linalg.norm(normal_point - b) == np.linalg.norm(a - b):
            normal_point = b
            a = map[(i+1)%len(map)] 
            b = map[(i+2)%len(map)] 
            direction = b - a

        d = np.linalg.norm(normal_point - future_pos)

        if d < closest_dist:
            closest_dist = d
            normal = normal_point
            
            direction = direction / np.linalg.norm(direction)
            direction *= 2
            target = normal_point + direction

    # calculate steering angle
    steering_angle = 0
    if closest_dist > PATH_RADIUS:
        steering_angle = np.arctan2(target[1] - y, target[0] - x)

    # publish to cmd_vel
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_SPEED / 100
    twist_msg.angular.z = steering_angle
    
    global vel_pub
    vel_pub.publish(twist_msg)


def main():
    rclpy.init(args=argv)

    # node initialization
    global node
    node = rclpy.create_node('controller')
    
    pos_sub = node.create_subscription(PoseStamped, 'position', position_callback, 10)
    global vel_pub
    vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    rate = node.create_rate(100)  # frequency in Hz
    rate, pos_sub, vel_pub
    node.get_logger().info('controller node launched.')

    rclpy.spin(node)

if __name__ == "__main__":
    main()