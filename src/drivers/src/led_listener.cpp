#include <vector>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "jetson_gpio.hpp"

class Led
{
public:
    Led(int red_pin, int green_pin, int blue_pin)
    {
        this->red = JetsonGPIO(red_pin, JetsonGPIO::OUTPUT, JetsonGPIO::HIGH);
        this->green = JetsonGPIO(green_pin, JetsonGPIO::OUTPUT, JetsonGPIO::HIGH);
        this->blue = JetsonGPIO(blue_pin, JetsonGPIO::OUTPUT, JetsonGPIO::HIGH);

        std::cout << this->red.pin << std::endl;
    }

    void setColor(bool r, bool g, bool b)
    {
        this->red.setValue(r ? JetsonGPIO::LOW : JetsonGPIO::HIGH);
        this->green.setValue(g ? JetsonGPIO::LOW : JetsonGPIO::HIGH);
        this->blue.setValue(b ? JetsonGPIO::LOW : JetsonGPIO::HIGH);
    }

private:
    JetsonGPIO red;
    JetsonGPIO green;
    JetsonGPIO blue;
};


class LedListener : public rclcpp::Node
{

public:
    LedListener() : Node("led_listener")
    {
        // parse parameters
        std::vector<std::string> led_topics;
        std::vector<std::tuple<int, int, int>> led_pins;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["sensors"]["led"]["topics"] >> led_topics;
        for (size_t i = 0; i < led_topics.size(); i++)
        {
            int r, g, b;
            fs["sensors"]["led"]["pins"]["red"][i] >> r;
            fs["sensors"]["led"]["pins"]["green"][i] >> g;
            fs["sensors"]["led"]["pins"]["blue"][i] >> b;
            led_pins.push_back(std::make_tuple(r, g, b));
        }
        fs.release();

        // initialize LEDs
        for (std::tuple<int, int, int> pin : led_pins)
        {
            leds.push_back(Led(std::get<0>(pin), std::get<1>(pin), std::get<2>(pin)));
        }
        
        // initialize subscribers
        for (size_t i = 0; i < led_topics.size(); i++)
        {
            this->create_subscription<std_msgs::msg::ColorRGBA>(led_topics[i], 10, std::bind(&LedListener::led_callback, this, std::placeholders::_1, i));
        }
    }

private:
    std::vector<Led> leds;

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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedListener>());
    rclcpp::shutdown();

    return 0;
}