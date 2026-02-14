#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geodesy/utm.h"
#include "gps_waypoint.hpp"
#include <deque>

class yet_another_gps_publisher : public rclcpp::Node
{
public:
    explicit yet_another_gps_publisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Parameters
    double min_spline_length_;
    std::string odom_topic_;
    std::string waypoint_file_topic_;
    std::string utm_frame_id_;
    std::string odom_frame_id_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer for periodic spline generation
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Current robot pose in odom frame (from odometry)
    geometry_msgs::msg::Pose current_pose_;

    // List of pending waypoints (already transformed to odom)
    std::deque<gps_waypoint> waypoints_;

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypoint_file_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();

    // Helper: transform a waypoint from lat/lon to odom frame using TF
    bool transformWaypoint(gps_waypoint& wp);
};