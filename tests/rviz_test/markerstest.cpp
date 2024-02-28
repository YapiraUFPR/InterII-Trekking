#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

geometry_msgs::msg::PoseStamped current_pose;
geometry_msgs::msg::PoseStamped dest_pose;

void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose = *msg;
}

void getlocalpose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    dest_pose = *msg;
}

int main(int argc, char **argv)
{
    //
    // ROS INITIALIZATION
    //
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("markerpoints");

    auto local_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "topic_path", 10, local_pos_cb);

    auto getlocalpose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "topic_path", 10, getlocalpose_cb);

    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    rclcpp::Rate loop_rate(2);

    visualization_msgs::msg::Marker points, line_strip, landmark;
    points.header.frame_id = line_strip.header.frame_id = landmark.header.frame_id = "map";
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    points.header.stamp = line_strip.header.stamp = landmark.header.stamp = clock->now();
    points.ns = line_strip.ns = landmark.ns = "points_and_lines";
    points.action = line_strip.action = landmark.action = visualization_msgs::msg::Marker::ADD;
    points.id = 0;
    line_strip.id = 1;
    landmark.id = 2;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST; // POINTS, SPHERE_LIST
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    landmark.type = visualization_msgs::msg::Marker::POINTS;
    points.pose.orientation.w = line_strip.pose.orientation.w = landmark.pose.orientation.w = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05; // 0.05, 0.1
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;
    line_strip.scale.x = 0.03;

    landmark.scale.x = 0.1;
    landmark.scale.y = 0.1;
    landmark.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Landmark is red
    landmark.color.r = 1.0;
    landmark.color.a = 1.0;
	
	
    geometry_msgs::msg::PoseStamped fake_pose;
    fake_pose.header.frame_id = "map";
    fake_pose.pose.position.x = 0.0; 
    fake_pose.pose.position.y = 0.0;  
    fake_pose.pose.position.z = 0.0;  
    
    dest_pose.pose.position.x = 1.0;
    dest_pose.pose.position.y = 1.0;
    dest_pose.pose.position.z = 1.0;
    
    while (rclcpp::ok())
    {
    	fake_pose.pose.position.x += 0.1;
    	fake_pose.pose.position.y += 0.1;
    	fake_pose.pose.position.z += 0.1;
    	
        points.points.push_back(fake_pose.pose.position);
        line_strip.points.push_back(fake_pose.pose.position);
        marker_pub->publish(points);
        marker_pub->publish(line_strip);

        if (dest_pose.pose.position.x && dest_pose.pose.position.y && dest_pose.pose.position.z)
        {
            landmark.points.push_back(dest_pose.pose.position);
            marker_pub->publish(landmark);
        }
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }

    return 0;
}
