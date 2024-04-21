#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <jetson-utils/videoSource.h>
#include "image_converter.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

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
        // std::string gst_pipeline = this->gstreamerPipeline(input_stream, cv::Size(resolution[0], resolution[1]), sample_rate, 0);
        // RCLCPP_INFO(this->get_logger(), "Gstreamer pipeline:\n%s", gst_pipeline.c_str());
        // this->cap = cv::VideoCapture(gst_pipeline, cv::CAP_GSTREAMER);
        // this->cap.open(input_stream);
        // if (!this->cap.isOpened())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
        // }

        videoOptions video_options;

        video_options.width = 640;
        video_options.height = 480;
        video_options.latency = 1;

        this->cap = videoSource::Create("csi://0", video_options);
        this->image_cvt = new imageConverter();

        RCLCPP_INFO(this->get_logger(), "Camera publisher has been started.");

        // initialize publisher
        this->publisher = this->create_publisher<sensor_msgs::msg::Image>(camera_topic, 10);
        this->timer = this->create_wall_timer(std::chrono::milliseconds(1000 / sample_rate), std::bind(&ImxPublisher::publish, this));
    }

private:
    std::string gstreamerPipeline(int sensor_id, cv::Size img_size, int frame_rate, int flip_method)
    {
        const char *fmt = "nvarguscamerasrc sensor-id=%d \
            ! video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=%d/1, format=(string)NV12 \
            ! nvvidconv flip-method=%d \
            ! video/x-raw(memory:NVMM) \
            ! ! nvegltransform ! nveglglessink -e";

        int sz = std::snprintf(nullptr, 0, fmt, std::sqrt(2));
        std::vector<char> buf(sz + 1); // note +1 for null terminator
        std::sprintf(buf.data(), fmt, 
             sensor_id, img_size.width, img_size.height, frame_rate, flip_method);

        return std::string(buf.data());
    }

    void publish(void)
    {
        // cv::Mat frame;
        // std::vector<uchar> buffer;

        sensor_msgs::msg::Image msg;
        imageConverter::PixelType* nextFrame = NULL;

        // get the latest frame
        if(!this->cap->Capture(&nextFrame, 1000) )
        {
            ROS_ERROR("failed to capture next frame");
            return;
        }

        // assure correct image size
        if(!this->image_cvt->Resize(this->cap->GetWidth(), this->cap->GetHeight(), imageConverter::ROSOutputFormat))
        {
            ROS_ERROR("failed to resize camera image converter");
            return;
        }

        // convert the image to ROS format
        this->image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame);


        // get frame from camera
        // cap >> frame;
        // if (frame.empty())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Could not get frame");
        // }
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing camera frames...");

        // convert and publish frame
        msg.header.stamp = this->now();
        // msg.format = "jpeg";
        // cv::imencode(".jpeg", frame, buffer);
        // msg.data = buffer;
        publisher->publish(msg);
    }

    videoSource* cap; 
    imageConverter* image_cvt;
    // cv::VideoCapture cap;
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