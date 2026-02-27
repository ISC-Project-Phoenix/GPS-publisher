#pragma once

#include <deque>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geodesy/utm.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class yet_another_gps_publisher : public rclcpp::Node {
public:
    explicit yet_another_gps_publisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Parameters
    double min_spline_length;
    std::string odom_topic;
    std::string waypoint_file_topic;
    std::string utm_frame_id;
    std::string odom_frame_id;
    std::string waypoint_file_path;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    // Timer for periodic spline generation
    rclcpp::TimerBase::SharedPtr timer;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Current robot pose in odom frame (from odometry)
    geometry_msgs::msg::Pose current_pose;

    // List of pending waypoints (already transformed to odom)
    std::deque<gps_waypoint> waypoints;

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    // helper to load waypoints from file
    bool load_waypoints(const std::string& file_path);

    // Helper: transform a waypoint from lat/lon to odom frame using TF
    bool transformWaypoint(gps_waypoint& wp);
};