#pragma once

#include <list>  /* for doubly linked lists*/
#include <string>
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
    // Parameters for localization and topics.
    std::string odom_topic;
    std::string utm_frame_id;
    std::string odom_frame_id;
    std::string map_frame_id;
    // parameters for waypoint file loading
    std::string waypoint_file_path;
    std::string waypoint_file_topic;
    // parameters for spline configs
    double min_spline_length;
    /// Parameters for confidence
    double max_gps_variance;  // Threshold in meters squared
    bool is_gps_valid = false;
    bool do_gps_variance_check = false;

    double arrival_threshold = 2.0;  // Meters

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub; /* local ekf */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_file_sub; 

    // GPS subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub; /* global ekf */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr raw_gps_sub; /* raw gps points*/

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub; /* publishes calculated spline */

    // Timer unused ircc
    rclcpp::TimerBase::SharedPtr timer; /* periodic callback (check if robot reached waypoint)*/

    // TF
    tf2_ros::Buffer tf_buffer_;               /* local cache that stores used as lookup for transforms */
    tf2_ros::TransformListener tf_listener_;  /* listens to broadcasts (utm, map, baselink)*/

    // Current robot pose (from odometry)
    geometry_msgs::msg::Pose current_pose{}; 

    // Doubly linked list of waypoints
    std::list<gps_waypoint> waypoints;

    // Iterator to next target waypoint
    std::list<gps_waypoint>::iterator current_waypoint_it_;

    // Helper to advance the iterator safely
    void advance_to_next_waypoint();

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    // GPS specific Callbacks
    void global_ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void raw_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // helper to load waypoints from file
    bool load_waypoints(const std::string& file_path);

    // Helper: transform a waypoint from lat/lon to odom frame using TF
    // unused ircc
    bool transformWaypoint(gps_waypoint& wp);
};