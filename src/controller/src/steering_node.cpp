#include <string>
#include <vector>
#include <fstream>
#include <tuple>
#include <chrono>
#include <unistd.h>
#include <deque>

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

    enum ROBOT_STATE
    {
        WAITING_FOR_BT,
        FIRST_MARK,
        NAVIGATING,
        SECOND_MARK,
        FAILSAFE,
    };

    std::vector<Eigen::Vector2d> map;

    nav_msgs::msg::Odometry current_pose;
    double current_speed;
    double current_angle;
    bool use_bt;

    // Callback queues
    const size_t BT_QUEUE_SIZE = 10;
    std::deque<std::string> bt_buttons;
    const size_t ODOM_QUEUE_SIZE = 300;
    std::deque<Eigen::Vector2d> odom_positions;
    std::deque<double> odom_angles;
    bool found_mark = false;
    const size_t CONE_QUEUE_SIZE = 10;
    std::deque<double> cone_errors;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mark_sub;
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

        this->bt_buttons.push_back(msg->data);
        if (this->bt_buttons.size() > this->BT_QUEUE_SIZE)
            this->bt_buttons.pop_front();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received odometry message: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);

        Eigen::Vector2d pose_2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
        this->odom_positions.push_back(pose_2d);
        if (this->odom_positions.size() > this->ODOM_QUEUE_SIZE)
            this->odom_positions.pop_front();
        
        this->odom_angles.push_back(msg->pose.pose.orientation.z);
        if (this->odom_angles.size() > this->ODOM_QUEUE_SIZE)
            this->odom_angles.pop_front();
    }

    void mark_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
            RCLCPP_INFO(this->get_logger(), "Found mark");

        this->found_mark = msg->data;
    }

    void cone_det_callback(const vision_msgs::msg::Detection2D::SharedPtr msg)
    {
        this->cone_errors.push_back(msg->results[0].pose.pose.position.x);
        if (this->cone_errors.size() > this->CONE_QUEUE_SIZE)
            this->cone_errors.pop_front();
    }

    // Publishers functions
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

    void setFlare(bool flare_state)
    {
        std_msgs::msg::Bool msg;
        msg.data = flare_state;
        this->flare_pub->publish(msg);
    }

    void setSpeed(double angle, double speed)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed / 100;
        msg.angular.z = angle;
        this->motors_pub->publish(msg);
    }

    Eigen::Vector2d estimateFuturePos(Eigen::Vector2d curr_pos, double curr_angle)
    {
        // TODO: better estimation for distance
        double distance = this->VELOCITY;

        Eigen::Vector2d future_pos;
        future_pos.x() = curr_pos.x() + distance * cos(curr_angle);
        future_pos.y() = curr_pos.y() + distance * sin(curr_angle);

        return future_pos;
    }

    double steer(Eigen::Vector2d curr_pos, double curr_angle)
    {
        Eigen::Vector2d normal, target;
        double closest_distance = std::numeric_limits<double>::infinity();

        Eigen::Vector2d future_pos = this->estimateFuturePos(curr_pos, curr_angle);

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

        RCLCPP_INFO(this->get_logger(), "Closest distance: %f", closest_distance);

        // Calculate angle to target
        double steering_angle = 0.0;
        if (closest_distance < this->PATH_RADIUS)
        {
            steering_angle = atan2(target.y() - this->current_pose.pose.pose.position.y, target.x() - this->current_pose.pose.pose.position.x);
        }

        return steering_angle;
    }

    double correctToCones(double error)
    {
        return error * 0.0045;
    }

public:
    SteeringNode(void) : Node("steering_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting steering node...");

        // Parse parameters
        cv::FileStorage fs("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        std::string odometry_topic = fs["steering"]["odometry_topic"].string();
        std::string map_topic = fs["steering"]["map_topic"].string();
        std::string map_file = fs["steering"]["map_file"].string();
        std::string mark_topic = fs["mark_detector"]["topic"].string();
        std::string flare_topic = fs["actuators"]["flare"]["topic"].string();
        std::string motors_topic = fs["actuators"]["motors"]["topic"].string();
        std::string cone_det_topic = fs["cone_detector"]["topic"].string();

        std::vector<std::string> led_topics;
        for (size_t i = 0; i < fs["actuators"]["led"]["topics"].size(); i++)
        {
            std::string led_topic = fs["actuators"]["led"]["topics"][i].string();
            led_topics.push_back(led_topic);
        }

        // Bluetooth start/stop
        fs["steering"]["bt_start_stop"] >> this->use_bt;
        if (use_bt)
        {
            std::string bt_topic = fs["sensors"]["bluetooth_gamepad"]["topic"].string();
            this->bt_sub = this->create_subscription<std_msgs::msg::String>(bt_topic, 10, std::bind(&SteeringNode::bt_callback, this, std::placeholders::_1));
        }

        // Parse values
        fs["steering"]["ignore_mark_for"] >> this->IGNORE_MARK_FOR;
        fs["steering"]["use_cone_pos_after"] >> this->USE_CONE_POS_AFTER;
        fs["steering"]["linear_speed"] >> this->LINEAR_SPEED;
        fs["steering"]["track_radius"] >> this->PATH_RADIUS;

        fs.release();

        // Create subscribers
        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10, std::bind(&SteeringNode::odom_callback, this, std::placeholders::_1));
        this->mark_sub = this->create_subscription<std_msgs::msg::Bool>(mark_topic, 10, std::bind(&SteeringNode::mark_callback, this, std::placeholders::_1));
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
    }

    void navigate(void)
    {
        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(get_node_base_interface());

        ROBOT_STATE state = this->use_bt ? ROBOT_STATE::WAITING_FOR_BT : ROBOT_STATE::FIRST_MARK;

        long steering_duration = 0;
        long ignore_mark_until = 0;
        double correction;

        bool exit = false;
        while (!exit && rclcpp::ok())
        {
            this->executor->spin_once(std::chrono::milliseconds(100));
            long curr_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            switch (state)
            {
            case ROBOT_STATE::WAITING_FOR_BT:
                this->setLeds({1, 0, 0}, {1, 0, 0});
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for bluetooth signal...");

                for (std::string bt_button : this->bt_buttons)
                {
                    // Start robot
                    if (bt_button == "Y")
                    {
                        state = ROBOT_STATE::FIRST_MARK;
                        break;
                    }

                    // Test flare
                    if (bt_button == "X")
                    {
                        this->setFlare(true);
                        sleep(2);
                        this->setFlare(false);

                        this->bt_buttons.clear();
                        break;
                    }
                }
                
                break;

            case ROBOT_STATE::FIRST_MARK:
                this->setLeds({0, 1, 0}, {0, 1, 0});
                RCLCPP_INFO(this->get_logger(), "Currently at first mark. Will begin navigation to second mark..");

                this->setSpeed(this->NEUTRAL_ANGLE, this->LINEAR_SPEED);
                                
                // Use steering algorithm for this amount of time
                steering_duration = curr_time + this->USE_CONE_POS_AFTER;

                // Do not check for mark for this amount of time
                ignore_mark_until = curr_time + this->IGNORE_MARK_FOR;

                state = ROBOT_STATE::NAVIGATING;

                break;

            case ROBOT_STATE::NAVIGATING:
                this->setLeds({0, 1, 1}, {0, 1, 1});
                RCLCPP_INFO_ONCE(this->get_logger(), "Navigating...");

                // Check if mark is found
                if ((curr_time > ignore_mark_until) && this->found_mark)
                {
                    RCLCPP_INFO(this->get_logger(), "Mark found.");
                    state = ROBOT_STATE::SECOND_MARK;
                    break;
                }   

                correction = 0.0;
                if ((curr_time < steering_duration) &&  (this->odom_positions.size() > 2))
                {
                    RCLCPP_INFO_ONCE(this->get_logger(), "Steering...");

                    // Steer
                    Eigen::Vector2d latest_pose = this->odom_positions.back();
                    this->odom_positions.pop_back();
                    double latest_angle = this->odom_angles.back();
                    this->odom_angles.pop_back();

                    correction = this->steer(latest_pose, latest_angle);
                }
                else if ((curr_time > steering_duration) && (this->cone_errors.size() > 2))
                {
                    RCLCPP_INFO_ONCE(this->get_logger(), "Using cone...");

                    // Use cone to correct position
                    double latest_error = this->cone_errors.back();
                    this->cone_errors.pop_back();

                    correction = this->correctToCones(latest_error);
                }
                RCLCPP_INFO(this->get_logger(), "Correction: %f", correction);
                this->setSpeed(correction, this->LINEAR_SPEED);

                // Check failsafe
                if (this->use_bt)
                {
                    for (std::string bt_button : this->bt_buttons)
                    {
                        if (bt_button == "A")
                        {
                            state = ROBOT_STATE::FAILSAFE;
                            break;
                        }
                    }
                }

                break;
            case ROBOT_STATE::SECOND_MARK:
                this->setLeds({0, 1, 0}, {0, 0, 0});
                RCLCPP_INFO(this->get_logger(), "Reached second mark. Exiting...");
                
                this->setSpeed(this->NEUTRAL_ANGLE, 0.0);
                exit = true;
                
                break;
            case ROBOT_STATE::FAILSAFE:
                this->setLeds({1, 0, 0}, {1, 0, 0});
                RCLCPP_INFO(this->get_logger(), "FAILSAFE ACTIVATED. Exiting...");
                
                this->setSpeed(this->NEUTRAL_ANGLE, 0.0);
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
