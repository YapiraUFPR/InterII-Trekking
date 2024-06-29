#include <string>
#include <vector>
#include <fstream>
#include <tuple>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "pid.h"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class SlamNode : public rclcpp::Node 
{
private:

    const double VELOCITY = 1.0;
    const double PATH_RADIUS = 5.0;
    const long FLARE_TIMEOUT = 200; // ms
    const double NEUTRAL_ANGLE = 0.0;
    const double MIN_ANGLE = -1.57;
    const double MAX_ANGLE = 1.57;

    const double DELTA_T = 1.0 / 138.0;

    int counter = 0;

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

    // PID controll
    double current_angular_speed;
    double current_robot_angle;
    double current_robot_distance;

    double ignore_first_mark;

    double ang_accel_min;

    // Motors controll
    double current_angle;
    double current_speed;

    double linear_speed;

    double kp, kd, ki;
    int ignore_first_n;
    
    // PID shared ptr
    std::shared_ptr<PID> pid;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flare_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motors_pub;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received IMU message: x=%f, y=%f, z=%f", msg->orientation.x, msg->orientation.y, msg->orientation.z);

        double accel_ang_z = msg->angular_velocity.z;

        // High pass filter
        accel_ang_z = (abs(accel_ang_z) < this->ang_accel_min) ? 0.0 : accel_ang_z;

        this->current_angular_speed += accel_ang_z * this->DELTA_T;
        this->current_robot_angle += accel_ang_z * this->DELTA_T;

        this->current_robot_distance += msg->linear_acceleration.x * this->DELTA_T * this->DELTA_T;

        this->imu_pid();
    }

    void imu_pid(void)
    {

        if (this->current_robot_angle > this->MAX_ANGLE)
            this->current_robot_angle = this->MAX_ANGLE;
        else if (this->current_robot_angle < this->MIN_ANGLE)
            this->current_robot_angle = this->MIN_ANGLE;

        double control = this->pid->calculate(this->NEUTRAL_ANGLE, this->current_robot_angle);

        // double control = this->NEUTRAL_ANGLE - this->current_robot_angle;

        if (++counter == 30)
        {
            RCLCPP_INFO(this->get_logger(), "Current ang speed: %f, Current angle: %f, Control %f, Linear distance: %f", this->current_angular_speed, this->current_robot_angle, control, this->current_robot_distance);

        }

        // if (ignore_first_n && ignore_first_n-- > 0)
        //     return;

        this->set_speed(this->current_angle - control, this->linear_speed);
    }

    void color_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received color message: r=%f, g=%f, b=%f, a=%f", msg->r, msg->g, msg->b, msg->a);

        if (this->current_robot_distance < this->ignore_first_mark)
            return;

        this->found_mark = (msg->r >= this->mark_color_lower[0] && msg->r <= this->mark_color_upper[0]) &&
                          (msg->g >= this->mark_color_lower[1] && msg->g <= this->mark_color_upper[1]) &&
                          (msg->b >= this->mark_color_lower[2] && msg->b <= this->mark_color_upper[2]);

        this->set_flare(found_mark);
    }

    void set_speed(double angle, double speed)
    {
        if (speed > 1.0)
            this->current_speed = 1.0;
        else if (speed < -1.0)
            this->current_speed = -1.0;
        else
            this->current_speed = speed;

        if (angle > this->MAX_ANGLE)
            this->current_angle = this->MAX_ANGLE;
        else if (angle < this->MIN_ANGLE)
            this->current_angle = this->MIN_ANGLE;
        else
            this->current_angle = angle;

        if (this->counter == 30)
        {
            RCLCPP_INFO(this->get_logger(), "Setting speed: %f, angle: %f", this->current_speed, this->current_angle);
        
            counter = 0;
        }

        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed;
        msg.angular.z = angle + 1.57;
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

public:

    SlamNode(void) : Node("slam_node") 
    {
        RCLCPP_INFO(this->get_logger(), "Starting steering node...");

        // parse parameters
        std::string imu_topic, color_topic; // Subscribers
        std::string flare_topic, motors_topic; // Publishers
        std::vector<int> mark_colors(3);
        cv::FileStorage fs("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["imu"]["topic"] >> imu_topic;
        fs["sensors"]["color"]["topic"] >> color_topic;
        fs["actuators"]["flare"]["topic"] >> flare_topic;
        fs["actuators"]["motors"]["topic"] >> motors_topic;
        fs["slam"]["mark_color_lower"] >> mark_color_lower;
        fs["slam"]["mark_color_upper"] >> mark_color_upper;

        fs["slam"]["pid"]["kp"] >> kp;
        fs["slam"]["pid"]["kd"] >> kd;
        fs["slam"]["pid"]["ki"] >> ki;

        fs["slam"]["ignore_first_n"] >> this->ignore_first_n;
        fs["slam"]["ignore_first_mark"] >> this->ignore_first_mark;
        fs["slam"]["linear_speed"] >> this->linear_speed;
        fs["slam"]["ang_accel_min"] >> this->ang_accel_min;

        fs.release();

        // Create subscribers
        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&SlamNode::imu_callback, this, std::placeholders::_1));
        this->color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(color_topic, 10, std::bind(&SlamNode::color_callback, this, std::placeholders::_1));

        // Create publishers
        this->flare_pub = this->create_publisher<std_msgs::msg::Bool>(flare_topic, 10);
        this->motors_pub = this->create_publisher<geometry_msgs::msg::Twist>(motors_topic, 10);

        this->ROBOT_STATE = robot_state::FIRST_MARK;
        this->found_mark = false;
        this->flare_turn_off_time = 0.0;

        this->current_angular_speed = 0.0;
        this->current_robot_angle = this->NEUTRAL_ANGLE;

        this->current_speed = 0.0;
        this->current_angle = this->NEUTRAL_ANGLE;
        
        this->pid = std::make_shared<PID>(this->DELTA_T, this->MAX_ANGLE, this->MIN_ANGLE, this->kp, this->kd, this->ki);
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
                    RCLCPP_INFO(this->get_logger(), "Currently at first mark. Will begin navigation to second mark..");
                    this->ROBOT_STATE = robot_state::NAVIGATING;
                    break;
                case robot_state::NAVIGATING:
                    RCLCPP_INFO(this->get_logger(), "Navigating...");
                    this->set_speed(this->NEUTRAL_ANGLE, this->linear_speed);
                    while (!this->found_mark && rclcpp::ok())
                    {
                        this->executor->spin_once(std::chrono::milliseconds(100));
                    }
                    this->ROBOT_STATE = robot_state::SECOND_MARK;
                    break;
                case robot_state::SECOND_MARK:
                    RCLCPP_INFO(this->get_logger(), "Reached second mark. Exiting...");
                    this->set_speed(this->NEUTRAL_ANGLE, 0.0);
                    sleep(5);
                    this->set_flare(false);
                    exit = true;
                    break;
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    SlamNode steering_node;
    steering_node.navigate();
    rclcpp::shutdown();

    return 0;
}