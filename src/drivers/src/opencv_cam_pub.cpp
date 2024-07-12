#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

class ImxPublisher : public rclcpp::Node
{
public:
    ImxPublisher(void) : Node("opencv_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Starting camera publisher...");

        // parse parameters
        std::string camera_topic, input_stream;
        int sample_rate;
        std::vector<int> resolution;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["camera"]["topic"] >> camera_topic;
        fs["sensors"]["camera"]["sample_rate"] >> sample_rate;
        fs["sensors"]["camera"]["input_stream"] >> input_stream;
        fs["sensors"]["camera"]["resolution"] >> resolution;
        fs["sensors"]["camera"]["imu_sync"] >> this->imu_sync;
        fs.release();

        this->cap.open(0, cv::CAP_ANY);

        if (!this->cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
        }


        RCLCPP_INFO(this->get_logger(), "Camera publisher has been started.");

        // initialize publisher
        this->publisher = this->create_publisher<sensor_msgs::msg::Image>(camera_topic, 10);
        this->timer = this->create_wall_timer(std::chrono::milliseconds(1000 / sample_rate), std::bind(&ImxPublisher::publish, this));
    }

private:

    void publish(void)
    {
        
        cv::Mat frame;
        this->cap.read(frame);

        cv::resize(frame, frame, cv::Size(640, 480));

        sensor_msgs::msg::Image msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());

        publisher->publish(msg);
    }

    cv::VideoCapture cap; 
    bool imu_sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char *argv[])
{
    // initialize ros2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImxPublisher>());
    rclcpp::shutdown();
    return 0;
}