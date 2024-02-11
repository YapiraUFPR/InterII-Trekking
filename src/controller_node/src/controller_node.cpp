#include <memory>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(): Node("controller")
    {
        // parse parameters
        std::string pose_topic;
        int sample_rate, input_stream;
        std::vector<int> resolution;
        cv::FileStorage fs;
        fs.open("/home/gab/projetos/yapira/bedman-trekker/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["camera"]["node"] >> node_name;
        fs["sensors"]["camera"]["topic"] >> camera_topic;
        fs["sensors"]["camera"]["sample_rate"] >> sample_rate;
        fs["sensors"]["camera"]["input_stream"] >> input_stream;
        fs["sensors"]["camera"]["resolution"] >> resolution;
        fs.release();

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}