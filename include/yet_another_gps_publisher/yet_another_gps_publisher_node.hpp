#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// yet_another_gps_publisher_node.cpp
class yet_another_gps_publisher : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

public:
    yet_another_gps_publisher(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void sub_cb(std_msgs::msg::String::SharedPtr msg);
};
