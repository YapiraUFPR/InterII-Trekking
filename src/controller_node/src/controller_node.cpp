#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/opencv.hpp>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(): Node("controller_node")
    {
        // parse parameters
        std::string poseTopic, flareTopic, colorTopic, motorsTopic;
        cv::FileStorage fs;
        fs.open("/home/gab/projetos/yapira/bedman-trekker/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["color"]["topic"] >> colorTopic;
        fs["actuators"]["flare"]["topic"] >> resolution;
        fs["actuators"]["motors"]["topic"] >> motorsTopic;
        fs.release();

        // Subscribe to pose topic    
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr poseSub = this->create_subscription<geometry_msgs::PoseStamped>(poseTopic, 10, std::bind(&ControllerNode::pose_callback, this, _1));

        // Create publishers
        this->motorPub = rclcpp::create_publisher<geometry_msgs::Twist>(motorsTopic, 10);
        this->flarePub = rclcpp::create_publisher<geometry_msgs::
    }

private:
    void topic_callback(const geometry_msgs::PoseStamped::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}