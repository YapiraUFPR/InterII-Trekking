#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "jetson_gpio.hpp"

class FlareListener : public rclcpp::Node
{

public:
    FlareListener() : Node("flare_listener")
    {
        RCLCPP_INFO(this->get_logger(), "Starting LED listener...");

        // parse parameters
        std::string flare_topic;
        int flare_pin;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["actuators"]["flare"]["topic"] >> flare_topic;
        fs["actuators"]["flare"]["pin"] >> flare_pin;
        fs.release();

        // Initialize led tape
        this->flare_gpio = JetsonGPIO(flare_pin, JetsonGPIO::Direction::OUTPUT, JetsonGPIO::Value::LOW);        

        // initialize subscribers
        this->flare_subscriber = this->create_subscription<std_msgs::msg::Bool>( flare_topic, 10, std::bind(&FlareListener::flare_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Flare listener has been started.");
    }

private:
    JetsonGPIO flare_gpio;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flare_subscriber;

    void flare_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message: %d", msg->data);
        if (msg->data)
        {
            this->flare_gpio.setValue(JetsonGPIO::Value::HIGH);
        } else 
        {
            this->flare_gpio.setValue(JetsonGPIO::Value::LOW);
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlareListener>());
    rclcpp::shutdown();

    return 0;
}