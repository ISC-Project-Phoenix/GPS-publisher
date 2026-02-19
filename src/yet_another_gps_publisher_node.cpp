#include "yet_another_gps_publisher/yet_another_gps_publisher_node.hpp"

#include <cmath>
#include <fstream>
#include <geographic_msgs/msg/geo_point.hpp>
#include <memory>
#include <sstream>

#include "yet_another_gps_publisher/spline_factory.hpp"

using namespace std::placeholders;

// Constructor
yet_another_gps_publisher::yet_another_gps_publisher(const rclcpp::NodeOptions& options)
    : Node("yet_another_gps_publisher", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Declare parameters
    min_spline_length_ = this->declare_parameter<double>("min_spline_length", 10.0);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    utm_frame_id_ = this->declare_parameter<std::string>("utm_frame_id", "utm");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    // TODO actually set this parameter from launch file or command line, not hardcoded.
    waypoint_file_path = this->declare_parameter<std::string>("waypoint_file", "waypoints.txt");

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&yet_another_gps_publisher::odom_callback, this, _1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("spline_path", 10);

    // Timer (1 Hz)
    timer_ =
        this->create_wall_timer(std::chrono::seconds(1), std::bind(&yet_another_gps_publisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "yet_another_gps_publisher started");

    // Load waypoints directly on startup!
    load_waypoints(waypoint_file_path);
}

// Odom callback
void yet_another_gps_publisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
}

// Normal function to load waypoints from file on startup
bool yet_another_gps_publisher::load_waypoints(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double lon, lat, radius = 0.0;
        std::string spline_type;

        if (!(iss >> lon >> lat >> spline_type)) {
            RCLCPP_WARN(this->get_logger(), "Skipping malformed line %d", line_num);
            continue;
        }
        if (spline_type == "circle") {
            if (!(iss >> radius)) {
                RCLCPP_WARN(this->get_logger(), "Circle method on line %d missing radius, using default 0", line_num);
            }
        }

        gps_waypoint wp(lon, lat, spline_type, radius);

        // Transform waypoint to odom frame
        if (!transformWaypoint(wp)) {
            RCLCPP_WARN(this->get_logger(), "Skipping waypoint line %d due to transform failure", line_num);
            continue;
        }

        waypoints_.push_back(wp);
        RCLCPP_INFO(this->get_logger(), "Loaded waypoint %zu: spline_type=%s at (%.6f, %.6f)", waypoints_.size(),
                    spline_type.c_str(), lon, lat);
    }
    file.close();
    return true;
}

// Transform waypoint from lat/lon to odom
bool yet_another_gps_publisher::transformWaypoint(gps_waypoint& wp) {
    // Create a GeoPoint from lat/lon
    geographic_msgs::msg::GeoPoint geo;
    geo.latitude = wp.latitude();
    geo.longitude = wp.longitude();
    geo.altitude = 0.0;  // assume ground level; could be extended

    // Convert to UTM using geodesy
    geodesy::UTMPoint utm;
    geodesy::fromMsg(geo, utm);  // This populates easting, northing, zone, etc.

    geometry_msgs::msg::PoseStamped utm_pose;
    utm_pose.header.frame_id = utm_frame_id_;
    utm_pose.header.stamp = this->now();
    utm_pose.pose.position.x = utm.easting;
    utm_pose.pose.position.y = utm.northing;
    utm_pose.pose.position.z = 0.0;
    utm_pose.pose.orientation.w = 1.0;

    try {
        geometry_msgs::msg::PoseStamped odom_pose = tf_buffer_.transform(utm_pose, odom_frame_id_);
        wp.setOdomPose(odom_pose.pose);
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return false;
    }
}

// Timer callback: generate and publish spline
void yet_another_gps_publisher::timer_callback() {
    if (waypoints_.empty()) {
        return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = odom_frame_id_;
    path.header.stamp = this->now();

    // Start with current pose
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path.header;
    start_pose.pose = current_pose_;
    path.poses.push_back(start_pose);

    double cumulative_length = 0.0;
    // Temporary waypoint for current pose (method irrelevant)
    gps_waypoint current_wp;
    current_wp.setOdomPose(current_pose_);
    current_wp.setEnabled(true);

    size_t used_count = 0;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& wp = waypoints_[i];
        if (!wp.enabled()) continue;

        const gps_waypoint& start_ref = (i == 0) ? current_wp : waypoints_[i - 1];

        std::vector<geometry_msgs::msg::Pose> segment;
        try {
            segment = gps_waypoint_spline::SplineFactory::generate(wp.method(), start_ref, wp);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Spline generation failed for method %s: %s", wp.method().c_str(),
                         e.what());
            break;
        }

        // Add segment points (skip first to avoid duplicate with previous end)
        for (size_t j = 1; j < segment.size(); ++j) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose = segment[j];
            path.poses.push_back(ps);
        }

        // Update cumulative length
        for (size_t j = 1; j < segment.size(); ++j) {
            const auto& a = segment[j - 1].position;
            const auto& b = segment[j].position;
            cumulative_length += std::hypot(b.x - a.x, b.y - a.y);
        }

        used_count = i + 1;

        if (cumulative_length >= min_spline_length_) {
            break;
        }
    }

    if (cumulative_length >= min_spline_length_) {
        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published spline path, length = %.2f m using %zu waypoints", cumulative_length,
                    used_count);
        // Optionally remove used waypoints:
        // waypoints_.erase(waypoints_.begin(), waypoints_.begin() + used_count);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Path too short (%.2f < %.2f), not publishing", cumulative_length,
                     min_spline_length_);
    }
}

// gps_waypoint constructor implementation
gps_waypoint::gps_waypoint(double lon, double lat, const std::string& method, double radius)
    : longitude_(lon), latitude_(lat), method_(method), radius_(radius), enabled_(true) {}

// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(yet_another_gps_publisher)