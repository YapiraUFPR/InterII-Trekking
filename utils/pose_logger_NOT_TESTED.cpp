#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>
#include <string>

class PoseLogger : public rclcpp::Node
{
public:
    explicit PoseLogger(const std::string& output_file, const std::string& pose_topic)
    : Node("pose_logger"), output_file_(output_file), pose_topic_(pose_topic)
    {
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, std::bind(&PoseLogger::pose_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::ofstream file(output_file_, std::ios_base::app);
        if (file.is_open()) {
            file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ","
                 << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << "\n";
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::string output_file_;
    std::string pose_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <output_file> <pose_topic>" << std::endl;
        return 1;
    }
    auto node = std::make_shared<PoseLogger>(std::string(argv[1]), std::string(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
