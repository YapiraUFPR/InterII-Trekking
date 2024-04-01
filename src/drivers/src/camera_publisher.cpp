#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class ImxPublisher : public rclcpp::Node
{
public:
    ImxPublisher(void) : Node("imx_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Starting camera publisher...");

        // parse parameters
        std::string camera_topic;
        int sample_rate, input_stream;
        std::vector<int> resolution;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["camera"]["topic"] >> camera_topic;
        fs["sensors"]["camera"]["sample_rate"] >> sample_rate;
        fs["sensors"]["camera"]["input_stream"] >> input_stream;
        fs["sensors"]["camera"]["resolution"] >> resolution;
        fs.release();

        // initialize camera
        std::string gst_pipeline = this->gstreamerPipeline(input_stream, cv::Size(resolution[0], resolution[1]), 30, 0);
        RCLCPP_INFO(this->get_logger(), "Gstreamer pipeline:\n%s", gst_pipeline.c_str());
        cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);
        cap.open(input_stream);
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
        }
        RCLCPP_INFO(this->get_logger(), "Camera publisher has been started.");

        // initialize publisher
        publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(camera_topic, 10);
        rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(std::chrono::milliseconds(1000 / sample_rate), std::bind(&ImxPublisher::publish, this));
    }

private:
    std::string gstreamerPipeline(int sensor_id, cv::Size img_size, int frame_rate, int flip_method)
    {
        //  "nvarguscamerasrc sensor-id=%d ! "
        //  "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        //  "nvvidconv flip-method=%d ! "
        //  "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        //  "videoconvert ! "
        //  "video/x-raw, format=(string)BGR ! appsink"

        return "nvarguscamerasrc sensor_id=" + std::to_string(sensor_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(img_size.width) + ", height=(int)" + std::to_string(img_size.height) + ", framerate=(fraction)" + std::to_string(frame_rate) + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(img_size.width) + ", height=(int)" + std::to_string(img_size.height) + ", format=(string)BGRx, ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    void publish(void)
    {
        cv::Mat frame;
        std::vector<uchar> buffer;
        sensor_msgs::msg::CompressedImage msg;
        
        // get frame from camera
        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get frame");
        }
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing camera frames...");

        // convert and publish frame
        msg.header.stamp = this->now();
        msg.format = "jpeg";
        cv::imencode(".jpeg", frame, buffer);
        msg.data = buffer;
        publisher->publish(msg);
    }

    cv::VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
};

int main(int argc, char *argv[])
{
    // initialize ros2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImxPublisher>());
    rclcpp::shutdown();
    return 0;
}