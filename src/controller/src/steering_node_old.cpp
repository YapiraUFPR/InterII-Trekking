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
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class SteeringNode : public rclcpp::Node
{
private:
    const double VELOCITY = 1.0;
    const long FLARE_TIMEOUT = 200; // ms
    const double NEUTRAL_ANGLE = 1.57;

    double PATH_RADIUS = 0.01;
    int LINEAR_SPEED = 13; // percent
    int IGNORE_MARK_FOR = 5000;
    int USE_CONE_POS_AFTER = 5000;

    enum robot_state
    {
        WAITING_FOR_BT,
        FIRST_MARK,
        NAVIGATING,
        SECOND_MARK,
        FAILSAFE,
    } ROBOT_STATE;

    std::vector<Eigen::Vector2d> map;

    bool found_mark;
    double flare_turn_off_time;
    std::vector<int> mark_color_lower;
    std::vector<int> mark_color_upper;
    nav_msgs::msg::Odometry current_pose;
    double current_speed;
    double current_angle;
    bool use_bt;
    long ignore_mark_countdown;
    long use_cone_pos_countdown;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr color_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr cone_det_sub;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flare_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motors_pub;
    std::vector<rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr> leds_pub;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    // Callbacks
    void bt_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received bluetooth message: %s", msg->data.c_str());

        if (msg->data == "Y")
        {
            this->ROBOT_STATE = robot_state::FIRST_MARK;
        }
        else if (msg->data == "A")
        {
            this->ROBOT_STATE = robot_state::FAILSAFE;
        }
        else if (msg->data == "X")
        {
            // Test flare
            this->set_flare(true);
            sleep(2);
            this->set_flare(false);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received odometry message: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        this->current_pose = *msg;

        long time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if ((this->use_cone_pos_countdown == 0) || (this->use_cone_pos_countdown < time_now))
        {
            return;
        }

        if (this->ROBOT_STATE == robot_state::NAVIGATING)
        {
            this->steer();
        }
    }

    void color_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received color message: r=%f, g=%f, b=%f, a=%f", msg->r, msg->g, msg->b, msg->a);

        long time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if ((this->ignore_mark_countdown != 0) > (time_now < this->ignore_mark_countdown))
        {
            this->found_mark = false;
        }
        else
        {
            this->found_mark = msg->data;
        }

        this->set_flare(found_mark);
    }

    void correctToCones(double error)
    {

        long time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if ((this->use_cone_pos_countdown == 0) || (this->use_cone_pos_countdown > time_now))
        {
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "Started correcting to cone direction...");

        // int signal = (error > 0) ? 1 : -1;

        double angle = error * 0.0045;

        // RCLCPP_INFO(this->get_logger(), "Correcting to cone direction: angle=%f", angle);

        set_speed(angle, this->LINEAR_SPEED / 2);
    }

    void cone_det_callback(const vision_msgs::msg::Detection2D::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received cone detection message: x=%f, y=%f", msg->results[0].pose.pose.position.x);

        this->correctToCones(msg->results[0].pose.pose.position.x);
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
        RCLCPP_INFO_ONCE(this->get_logger(), "Started steering...");

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
            RCLCPP_INFO(this->get_logger(), "Curr pos: %f x %f, target: %f x %f", this->current_pose.pose.pose.position.x, this->current_pose.pose.pose.position.y, target.x(), target.y());
            steering_angle = atan2(target.y() - this->current_pose.pose.pose.position.y, target.x() - this->current_pose.pose.pose.position.x);
        }

        RCLCPP_INFO(this->get_logger(), "Correcting %f", steering_angle);

        // Publish steering angle
        // geometry_msgs::msg::Twist msg;
        // msg.linear.x = 0.0; // Keep velocity constant
        // msg.angular.z = steering_angle;
        // this->motors_pub->publish(msg);

        this->set_speed(steering_angle, 0.0);
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
        std::string odometry_topic, mark_topic, cone_det_topic; // Subscribers
        std::string map_topic, flare_topic, motors_topic;       // Publishers
        std::string map_file;
        std::vector<std::string> led_topics;
        std::vector<int> mark_colors(3);
        cv::FileStorage fs("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["steering"]["odometry_topic"] >> odometry_topic;
        fs["steering"]["map_topic"] >> map_topic;
        fs["steering"]["map_file"] >> map_file;
        fs["mark_detector"]["topic"] >> color_topic;
        fs["actuators"]["flare"]["topic"] >> flare_topic;
        fs["actuators"]["motors"]["topic"] >> motors_topic;
        fs["steering"]["mark_color_lower"] >> mark_color_lower;
        fs["steering"]["mark_color_upper"] >> mark_color_upper;
        fs["cone_detector"]["topic"] >> cone_det_topic;
        for (size_t i = 0; i < fs["actuators"]["led"]["topics"].size(); i++)
        {
            std::string led_topic;
            fs["actuators"]["led"]["topics"][i] >> led_topic;
            led_topics.push_back(led_topic);
        }

        // Bluetooth start/stop
        fs["steering"]["bt_start_stop"] >> this->use_bt;
        if (use_bt)
        {
            std::string bt_topic = fs["sensors"]["bluetooth_gamepad"]["topic"].string();
            this->bt_sub = this->create_subscription<std_msgs::msg::String>(bt_topic, 10, std::bind(&SteeringNode::bt_callback, this, std::placeholders::_1));
        }

        fs["steering"]["ignore_mark_for"] >> this->IGNORE_MARK_FOR;
        fs["steering"]["use_cone_pos_after"] >> this->USE_CONE_POS_AFTER;
        fs["steering"]["linear_speed"] >> this->LINEAR_SPEED;
        fs["steering"]["track_radius"] >> this->PATH_RADIUS;

        fs.release();

        // Create subscribers
        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&SteeringNode::odom_callback, this, std::placeholders::_1));
        this->color_sub = this->create_subscription<std_msgs::msg::Bool>(color_topic, 10, std::bind(&SteeringNode::color_callback, this, std::placeholders::_1));
        this->cone_det_sub = this->create_subscription<vision_msgs::msg::Detection2D>(cone_det_topic, 10, std::bind(&SteeringNode::cone_det_callback, this, std::placeholders::_1));

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
        }
        RCLCPP_INFO(this->get_logger(), "Map loaded with %d points", this->map.size());

        this->ROBOT_STATE = this->use_bt ? robot_state::WAITING_FOR_BT : robot_state::FIRST_MARK;
        this->found_mark = false;
        this->flare_turn_off_time = 0.0;
        this->ignore_mark_countdown = 0;
        this->use_cone_pos_countdown = 0;
    }

    void navigate(void)
    {
        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(get_node_base_interface());

        bool exit = false;
        while (!exit && rclcpp::ok())
        {
            switch (this->ROBOT_STATE)
            {
            case robot_state::WAITING_FOR_BT:
                this->setLeds({1, 0, 0}, {1, 0, 0});
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for bluetooth signal...");
                this->executor->spin_once(std::chrono::milliseconds(100));
                break;

            case robot_state::FIRST_MARK:
                this->setLeds({0, 1, 0}, {0, 1, 0});
                RCLCPP_INFO(this->get_logger(), "Currently at first mark. Will begin navigation to second mark..");
                this->ROBOT_STATE = robot_state::NAVIGATING;
                break;

            case robot_state::NAVIGATING:
                this->setLeds({0, 1, 1}, {0, 1, 1});
                RCLCPP_INFO_ONCE(this->get_logger(), "Navigating...");
                this->set_speed(this->NEUTRAL_ANGLE, this->LINEAR_SPEED);

                if (this->ignore_mark_countdown == 0)
                {
                    this->ignore_mark_countdown = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() + this->IGNORE_MARK_FOR;
                    RCLCPP_INFO(this->get_logger(), "Ignoring mark for %d ms...", this->IGNORE_MARK_FOR);
                }

                if (this->use_cone_pos_countdown == 0)
                {
                    this->use_cone_pos_countdown = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() + this->USE_CONE_POS_AFTER;
                    RCLCPP_INFO(this->get_logger(), "Will use cone pose after %d ms...", this->USE_CONE_POS_AFTER);
                }

                this->executor->spin_once(std::chrono::milliseconds(100));

                if (this->found_mark)
                {
                    this->set_speed(this->NEUTRAL_ANGLE, 0.0);
                    this->ROBOT_STATE = robot_state::SECOND_MARK;
                    this->ignore_mark_countdown = 0;
                }

                break;
            case robot_state::SECOND_MARK:
                this->setLeds({0, 1, 0}, {0, 0, 0});
                RCLCPP_INFO(this->get_logger(), "Reached second mark. Exiting...");
                this->set_speed(this->NEUTRAL_ANGLE, 0.0);
                exit = true;
                break;
            case robot_state::FAILSAFE:
                this->setLeds({1, 0, 0}, {1, 0, 0});
                RCLCPP_INFO(this->get_logger(), "FAILSAFE ACTIVATED. Exiting...");
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
