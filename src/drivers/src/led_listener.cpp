#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include <JetsonGPIO.h>

class Led
{
public:
    Led(int red_pin, int green_pin, int blue_pin)
    {
        GPIO::setup(red_pin, GPIO::OUT, GPIO::HIGH);
        GPIO::setup(green_pin, GPIO::OUT, GPIO::HIGH);
        GPIO::setup(blue_pin, GPIO::OUT, GPIO::HIGH);

        this->red = red;
        this->green = green;
        this->blue = blue;
    }

    void setColor(bool r, bool g, bool b)
    {
        GPIO::output(this->red, r ? GPIO::LOW : GPIO::HIGH);
        GPIO::output(this->green, g ? GPIO::LOW : GPIO::HIGH);
        GPIO::output(this->blue, b ? GPIO::LOW : GPIO::HIGH);
    }

private:
    int red;
    int green;
    int blue;
};


class LedListener : public rclcpp::Node
{

public:
    LedListener() : Node("led_listener")
    {
        RCLCPP_INFO(this->get_logger(), "Starting LED listener...");

        // parse parameters
        std::vector<std::string> led_topics;
        std::vector<std::tuple<int, int, int>> led_pins;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["actuators"]["led"]["topics"] >> led_topics;
        for (size_t i = 0; i < led_topics.size(); i++)
        {
            int r, g, b;
            fs["actuators"]["led"]["pins"]["red"][i] >> r;
            fs["actuators"]["led"]["pins"]["green"][i] >> g;
            fs["actuators"]["led"]["pins"]["blue"][i] >> b;
            led_pins.push_back(std::make_tuple(r, g, b));
        }
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Initializing %d LEDs...", led_topics.size());

        // initialize LEDs
        for (std::tuple<int, int, int> pin : led_pins)
        {
            leds.push_back(Led(std::get<0>(pin), std::get<1>(pin), std::get<2>(pin)));
        }
        
        // initialize subscribers
        for (size_t i = 0; i < led_topics.size(); i++)
        {
            std::function<void(const std_msgs::msg::ColorRGBA::SharedPtr msg)> callback = std::bind(&LedListener::led_callback, this, std::placeholders::_1, i);
            led_subscribers.push_back(this->create_subscription<std_msgs::msg::ColorRGBA>(led_topics[i], 10, callback));
        }

        RCLCPP_INFO(this->get_logger(), "LED listener has been started.");
    }

private:
    std::vector<Led> leds;
    std::vector<rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr> led_subscribers;

    void led_callback(const std_msgs::msg::ColorRGBA::SharedPtr msg, int led_idx)
    {
        bool r = msg->r > 0.0;
        bool g = msg->g > 0.0;
        bool b = msg->b > 0.0;

        RCLCPP_INFO(this->get_logger(), "Setting LED %d to RGB(%d %d %d)", led_idx, r, g, b);

        this->leds[led_idx].setColor(r, g, b);
    }
};

int main(int argc, char const *argv[])
{
    GPIO::setmode(GPIO::BCM);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedListener>());
    rclcpp::shutdown();

    return 0;
}