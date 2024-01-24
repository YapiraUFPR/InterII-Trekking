#include <vector>
#include <iostream>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

int main(int argc, char *argv[])
{
    // parse parameters
    std::string node_name, camera_topic;
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
    
    // initialize ros2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(node_name);
    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>(camera_topic, 10);
    auto loop_rate = rclcpp::WallRate(sample_rate);

    // initialize camera
    cv::VideoCapture cap;
    cap.open(input_stream);
    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Could not open camera");
    }
    RCLCPP_INFO(node->get_logger(), "Camera publisher has been started");

    cv::Mat frame;
    std::vector<uchar> buffer;
    sensor_msgs::msg::CompressedImage msg;
    while (rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "Could not get frame");
        }

        msg.header.stamp = node->now();
        msg.format = "jpeg";
        cv::imencode(".jpeg", frame, buffer);
        msg.data = buffer;
        publisher->publish(msg);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}