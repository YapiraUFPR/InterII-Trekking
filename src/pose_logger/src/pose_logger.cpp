#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

class PoseLogger : public rclcpp::Node
{
public:
    PoseLogger() : Node("pose_logger")
    {
        RCLCPP_INFO(this->get_logger(), "Starting pose logger...");

        // Parse parameters
        std::string pose_topic, output_path;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["pose_logger"]["pose_topic"] >> pose_topic;
        fs["pose_logger"]["output_file"] >> output_path;
        fs["pose_logger"]["skip_poses"] >> this->skip_poses;

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PoseLogger::pose_callback, this, std::placeholders::_1));

        this->output_file = std::ofstream(output_path, std::ios_base::app);
        if (!this->output_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output file %s", output_path.c_str());
            exit(1);
        }

        this->pose_counter = -1;

        RCLCPP_INFO(this->get_logger(), "Pose logger has been started.");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if ((++this->pose_counter % this->skip_poses) != 0) {
            return;
        }

        this->output_file << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << std::endl;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    int skip_poses, pose_counter;
    std::ofstream output_file;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseLogger>());
    rclcpp::shutdown();
    return 0;
}
