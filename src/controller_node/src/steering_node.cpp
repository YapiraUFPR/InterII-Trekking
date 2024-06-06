#include <string>
#include <vector>
#include <fstream>
#include <tuple>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class SteeringNode : public rclcpp::Node 
{
private:

    const double VELOCITY = 1.0;
    const double PATH_RADIUS = 5.0;
    const long FLARE_TIMEOUT = 200; // ms
    const long LINEAR_SPEED = 20; // percent
    const double NEUTRAL_ANGLE = 1.57;

    enum robot_state {
        FIRST_MARK,
        NAVIGATING,
        SECOND_MARK,
    } ROBOT_STATE;

    std::vector<Eigen::Vector2d> map;

    bool found_mark;
    double flare_turn_off_time;
    std::vector<int> mark_color_lower;
    std::vector<int> mark_color_upper;
    nav_msgs::msg::Odometry current_pose;
    double current_speed;
    double current_angle;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flare_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motors_pub;
    std::vector<rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr> leds_pub;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odometry message: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        this->current_pose = *msg;

        if (this->ROBOT_STATE == robot_state::NAVIGATING)
        {
            this->steer();
        }
    }

    void color_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received color message: r=%f, g=%f, b=%f, a=%f", msg->r, msg->g, msg->b, msg->a);

        this->found_mark = (msg->r >= this->mark_color_lower[0] && msg->r <= this->mark_color_upper[0]) &&
                          (msg->g >= this->mark_color_lower[1] && msg->g <= this->mark_color_upper[1]) &&
                          (msg->b >= this->mark_color_lower[2] && msg->b <= this->mark_color_upper[2]);

        this->set_flare(found_mark);
    }

    void set_speed(double angle, double speed)
    {
        this->current_speed = speed;
        this->current_angle = angle;

        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed;
        msg.angular.z = angle;
        this->motors_pub->publish(msg);
    }

    void set_flare(bool state)
    {
        if (state)
        {
            this->flare_turn_off_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() + this->FLARE_TIMEOUT;
        } 

        bool flare_state = state || (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() < this->flare_turn_off_time);

        std_msgs::msg::Bool msg;
        msg.data = flare_state;
        this->flare_pub->publish(msg);
    }

    Eigen::Vector2d estimate_future_position(void)
    {
        // TODO: better estimation for distance
        double distance = this->VELOCITY;

        Eigen::Vector2d future_pos;
        future_pos.x() = this->current_pose.pose.pose.position.x + distance * cos(this->current_pose.pose.pose.orientation.z);
        future_pos.y() = this->current_pose.pose.pose.position.y + distance * sin(this->current_pose.pose.pose.orientation.z);

        return future_pos;
    }

    void steer(void)
    {
        Eigen::Vector2d normal, target;
        double closest_distance = std::numeric_limits<double>::infinity();

        Eigen::Vector2d future_pos = this->estimate_future_position();

        for (size_t i = 0; i < this->map.size() - 1; i++)
        {
            // Line segment
            Eigen::Vector2d a = this->map[i];
            Eigen::Vector2d b = this->map[i + 1];

            // Calculate normal vector between line segment and future position
            Eigen::Vector2d ap = future_pos - a;
            Eigen::Vector2d ab = b - a;
            ab.normalize();

            Eigen::Vector2d normal_point = (ab * ap.dot(ab)) + a;
            Eigen::Vector2d direction = b - a;

            // Check if normal point is on line segment
            if ((normal_point - a).norm() + (normal_point - b).norm() == (a - b).norm()) 
            {
                normal_point = b;
                a = map[(i + 1) % map.size()];
                b = map[(i + 2) % map.size()];
                direction = b - a;
            }

            double d = (normal_point - future_pos).norm();

            if (d < closest_distance)
            {
                closest_distance = d;
                normal = normal_point;
                
                direction.normalize();
                direction *= 2;
                target = normal_point + direction;
            }
        }

        // Calculate angle to target
        double steering_angle = 0.0;
        if (closest_distance < this->PATH_RADIUS)
        {
            steering_angle = atan2(target.y() - this->current_pose.pose.pose.position.y, target.x() - this->current_pose.pose.pose.position.x);
        }
        
        // Publish steering angle
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0; // Keep velocity constant
        msg.angular.z = steering_angle;
        this->motors_pub->publish(msg);
    }

    void setLeds(std::vector<int> color, std::vector<int> color2)
    {
        std_msgs::msg::ColorRGBA msg;
        msg.r = color[0];
        msg.g = color[1];
        msg.b = color[2];
        msg.a = 1.0;
        this->leds_pub[0]->publish(msg);

        msg.r = color2[0];
        msg.g = color2[1];
        msg.b = color2[2];
        this->leds_pub[0]->publish(msg);
    }

public:

    SteeringNode(void) : Node("steering_node") 
    {
        RCLCPP_INFO(this->get_logger(), "Starting steering node...");

        // parse parameters
        std::string odometry_topic, color_topic; // Subscribers
        std::string map_topic, flare_topic, motors_topic; // Publishers
        std::string map_file;
        std::vector <std::string> led_topics;
        std::vector<int> mark_colors(3);
        cv::FileStorage fs("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["steering"]["odometry_topic"] >> odometry_topic;
        fs["steering"]["map_topic"] >> map_topic;
        fs["steering"]["map_file"] >> map_file;
        fs["sensors"]["color"]["topic"] >> color_topic;
        fs["actuators"]["flare"]["topic"] >> flare_topic;
        fs["actuators"]["motors"]["topic"] >> motors_topic;
        fs["steering"]["mark_color_lower"] >> mark_color_lower;
        fs["steering"]["mark_color_upper"] >> mark_color_upper;
        for (size_t i = 0; i < fs["actuators"]["led"]["topics"].size(); i++)
        {
            std::string led_topic;
            fs["actuators"]["led"]["topics"][i] >> led_topic;
            led_topics.push_back(led_topic);
        }

        fs.release();

        // Create subscribers
        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&SteeringNode::odom_callback, this, std::placeholders::_1));
        this->color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(color_topic, 10, std::bind(&SteeringNode::color_callback, this, std::placeholders::_1));

        // Create publishers
        this->map_pub = this->create_publisher<nav_msgs::msg::Odometry>(map_topic, 10);
        this->flare_pub = this->create_publisher<std_msgs::msg::Bool>(flare_topic, 10);
        this->motors_pub = this->create_publisher<geometry_msgs::msg::Twist>(motors_topic, 10);
        for (std::string led_topic : led_topics)
        {
            this->leds_pub.push_back(this->create_publisher<std_msgs::msg::ColorRGBA>(led_topic, 10));
        }

        // Load map
        RCLCPP_INFO(this->get_logger(), "Loading map from %s", map_file.c_str());
        std::ifstream file(map_file);
        for (std::string line; std::getline(file, line);)
        {
            std::istringstream iss(line);
            
            float x, y;
            iss >> x >> y;
            this->map.push_back(Eigen::Vector2d(x, y));

            // Publish map for visualization
            nav_msgs::msg::Odometry msg;
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            this->map_pub->publish(msg);

            sleep(1);
        }

        this->ROBOT_STATE = robot_state::FIRST_MARK;
        this->found_mark = false;
        this->flare_turn_off_time = 0.0;
    }

    void navigate(void)
    {
        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(get_node_base_interface());

        bool exit = false;
        while (!exit)
        {
            switch (this->ROBOT_STATE)
            {
                case robot_state::FIRST_MARK:
                    this->setLeds([0, 1, 0], [0, 1, 0]);
                    RCLCPP_INFO(this->get_logger(), "Currently at first mark. Will begin navigation to second mark..");
                    this->ROBOT_STATE = robot_state::NAVIGATING;
                    break;
                case robot_state::NAVIGATING:
                    this->setLeds([0, 1, 1], [0, 1, 1]);
                    RCLCPP_INFO(this->get_logger(), "Navigating...");
                    this->set_speed(this->NEUTRAL_ANGLE, this->LINEAR_SPEED);
                    while (!this->found_mark)
                    {
                        this->executor->spin_once(std::chrono::milliseconds(100));
                    }
                    this->ROBOT_STATE = robot_state::SECOND_MARK;
                    break;
                case robot_state::SECOND_MARK:
                    this->setLeds([0, 1, 0], [0, 0, 0]);
                    RCLCPP_INFO(this->get_logger(), "Reached second mark. Exiting...");
                    this->set_speed(this->NEUTRAL_ANGLE, 0.0);
                    exit = true;
                    break;
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    SteeringNode steering_node;
    steering_node.navigate();
    rclcpp::shutdown();

    return 0;
}