#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

const std::vector<std::vector<float>> COLORS = {
    {0.1, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 1.0},
};

class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode (void) : Node("visualization_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting visualization node...");

        // Parse parameters
        std::vector<std::string> visualization_topics;
        cv::FileStorage fs;
        fs.open("/home/user/ws/src/config/config.yaml", cv::FileStorage::READ);
        fs["visualization_node"]["topics"] >> visualization_topics;
        fs.release();
        
        // Intialize messages static parameters
        for (size_t i = 0; i < visualization_topics.size(); i++)
        {
            visualization_msgs::msg::Marker msg;
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            msg.ns = "points_and_lines";
            msg.action = visualization_msgs::msg::Marker::ADD;
            msg.id = i;
            msg.type = visualization_msgs::msg::Marker::LINE_STRIP; 
            msg.pose.orientation.w = 1.0;

            msg.scale.x = 0.3; 
            msg.scale.y = 0.3;
            msg.scale.z = 0.3;

            msg.color.r = COLORS[i][0];
            msg.color.g = COLORS[i][1];
            msg.color.b = COLORS[i][2];
            msg.color.a = 1.0;

            this->msgs.push_back(msg);
        }

        // Initialize subscribers
        for (size_t i = 0; i < visualization_topics.size(); i++)
        {
            std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> callback = std::bind(&VisualizerNode::pose_callback, this, std::placeholders::_1, i);
            this->pose_subscribers.push_back(this->create_subscription<nav_msgs::msg::Odometry>(visualization_topics[i], 10, callback));

            RCLCPP_INFO(this->get_logger(), ("/visualization" + visualization_topics[i]).c_str());
            this->markers_publishers.push_back(this->create_publisher<visualization_msgs::msg::Marker>("/visualization" + visualization_topics[i], 10));
        }

        RCLCPP_INFO(this->get_logger(), "Visuzalition node has been started.");
    }

private:
    
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int idx)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Started publishing markers...");

        this->msgs[idx].header.stamp = this->now();
        this->msgs[idx].points.push_back(msg->pose.pose.position);
        this->markers_publishers[idx]->publish(this->msgs[idx]);
    }

    std::vector<visualization_msgs::msg::Marker> msgs;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> pose_subscribers;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> markers_publishers;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();

    return 0;
}