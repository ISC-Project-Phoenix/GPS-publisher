#include "yet_another_gps_publisher/yet_another_gps_publisher_node.hpp"

#include <cmath>
#include <fstream>
#include <geographic_msgs/msg/geo_point.hpp>
#include <memory>
#include <sstream>
#include <chrono>

#include "yet_another_gps_publisher/spline_factory.hpp"

// Constructor
yet_another_gps_publisher::yet_another_gps_publisher(const rclcpp::NodeOptions& options)
    : Node("yet_another_gps_publisher", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Declare parameters

    // Threshold for GPS "Confidence". 0.1 means we only trust the GPS if it's within ~30cm precision.
    // TODO figure out what a good threshold is based on the actual GPS variance we see in testing, and maybe even make it adaptive based on current conditions.
    max_gps_variance = this->declare_parameter<double>("max_gps_variance", 0.1);

    // true for do GPS varance check false if not.
    do_gps_variance_check = this->declare_parameter<bool>("do_gps_variance_check", false);

    // This is the mimium size of the spline as required by the controls team. If its too short they cannot plan ahead of corners enough.
    min_spline_length = this->declare_parameter<double>("min_spline_length", 10.0);

    // this is the mimium radus for the kart to have considered "arrived" at a particular waypoint.
    // as is the norm for ROS2 this unit is in meters.
    arrival_threshold = this->declare_parameter<double>("arrival_threshold", 2.0);

    // why the odom topic is a parameter: in sim we use the filtered odometry from the sim, but on the real robot we might want to use a different topic or maybe even have it remapped from the sim topic to the real topic.
    odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    // This is the utm Frame. Keep in might dearborn and purdue have different utm zones, so this might be necessary to change when we switch between the two or where ever you are.
    utm_frame_id = this->declare_parameter<std::string>("utm_frame_id", "utm");
    // This is the odom frame we will translate the spline into.
    odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
    // this is the MAP frame that we will store the waypoints in over time.
    map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
    // TODO actually set this parameter from launch file or command line, not hardcoded.
    // TODO indentify where this file should be stored?
    waypoint_file_path = this->declare_parameter<std::string>("gps_points_files", "gps_waypoints_parking_lot_mk1.txt");

    // Publisher
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // Subscribers
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&yet_another_gps_publisher::odom_callback, this, std::placeholders::_1));

    // Subscribe to Raw GPS to check the fix status (VectorNav)
    raw_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/phoenix/navsat", 10, std::bind(&yet_another_gps_publisher::raw_gps_callback, this, std::placeholders::_1));

    // Subscribe to NavSat Transform output to trigger spline generation
    gps_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/gps", 10, std::bind(&yet_another_gps_publisher::gps_odom_callback, this, std::placeholders::_1));

    // Load waypoints and initialize iterator
    if (load_waypoints(waypoint_file_path)) {
        current_waypoint_it_ = waypoint_list_.begin();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints. Node will not publish paths.");
    }
}

// Odom callback
void yet_another_gps_publisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose = msg->pose.pose;
}

// The Confidence Check + RAW GPS callback
void yet_another_gps_publisher::raw_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // check if we are even using this
    if (!do_gps_variance_check) {
        is_gps_valid = true;
        return;
    }

    // Status < 0 means NO_FIX.
    // We also check the covariance (diagonal [0] is Easting, [7] is Northing)
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GPS Lost Fix!");
        is_gps_valid = false;
        return;
    }

    if (msg->position_covariance[0] > max_gps_variance) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GPS Variance too high: %f",
                             msg->position_covariance[0]);
        is_gps_valid = false;
        return;
    }

    is_gps_valid = true;
}

// Load waypoints from file into std::list
bool yet_another_gps_publisher::load_waypoints(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return false;
    }

    waypoint_list_.clear();
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

        waypoints.push_back(wp);
        RCLCPP_INFO(this->get_logger(), "Loaded waypoint %zu: spline_type=%s at (%.6f, %.6f)", waypoints.size(),
                    spline_type.c_str(), lon, lat);
    }
    file.close();
    return true;
}

// Transform waypoint from lat/lon to UTM and store
bool yet_another_gps_publisher::transformWaypoint(gps_waypoint& wp) {
    geographic_msgs::msg::GeoPoint geo;
    geo.latitude = wp.latitude();
    geo.longitude = wp.longitude();
    geo.altitude = 0.0;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(geo, utm);

    geometry_msgs::msg::PoseStamped utm_pose;
    utm_pose.header.frame_id = utm_frame_id;
    // Do NOT set the stamp here, we want the TF buffer to grab the newest available transform later
    utm_pose.pose.position.x = utm.easting;
    utm_pose.pose.position.y = utm.northing;
    utm_pose.pose.position.z = 0.0;
    utm_pose.pose.orientation.w = 1.0;

    // Save the UTM pose to the waypoint, but don't do the TF lookup yet
    wp.setUtmPose(utm_pose);
    return true;
}

// Advance iterator to next waypoint (safe)
void yet_another_gps_publisher::advance_to_next_waypoint() {
    if (current_waypoint_it_ != waypoint_list_.end()) {
        ++current_waypoint_it_;
    }
}

// GPS odom callback: generate spline path from robot to remaining waypoints
void yet_another_gps_publisher::gps_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Guard conditions
    if (!is_gps_valid || waypoint_list_.empty() || current_waypoint_it_ == waypoint_list_.end()) {
        return;
    }

    // Robot pose from GPS is in map frame
    geometry_msgs::msg::Pose robot_pose_map = msg->pose.pose;

    // Transform all remaining waypoints from UTM to map (we'll need them for arrival check and spline)
    std::vector<std::pair<gps_waypoint*, geometry_msgs::msg::Pose>> remaining_waypoints;
    for (auto it = current_waypoint_it_; it != waypoint_list_.end(); ++it) {
        try {
            it->utmPose().header.stamp = rclcpp::Time(0);
            geometry_msgs::msg::PoseStamped map_wp = tf_buffer_.transform(
                it->utmPose(), map_frame_id, std::chrono::milliseconds(100));
            remaining_waypoints.emplace_back(&(*it), map_wp.pose);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF UTM->MAP failed: %s", ex.what());
            return;
        }
    }

    // Skip waypoints that we've already arrived at
    while (!remaining_waypoints.empty()) {
        const auto& target_pose = remaining_waypoints[0].second;
        double dist_to_target = std::hypot(
            robot_pose_map.position.x - target_pose.position.x,
            robot_pose_map.position.y - target_pose.position.y);
        if (dist_to_target >= arrival_threshold) {
            break;  // first non-arrived waypoint found
        }
        // Arrived at this waypoint; advance and remove from list
        RCLCPP_INFO(this->get_logger(), "Passed waypoint (arrival distance %.2f < %.2f)",
                    dist_to_target, arrival_threshold);
        advance_to_next_waypoint();
        remaining_waypoints.erase(remaining_waypoints.begin());
    }

    // If no remaining waypoints, we're done
    if (remaining_waypoints.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "All waypoints reached.");
        return;
    }

    // Build path by daisy-chaining splines from robot to first waypoint, then between waypoints
    nav_msgs::msg::Path path_map;
    path_map.header.frame_id = map_frame_id;
    path_map.header.stamp = msg->header.stamp;

    double cumulative_length = 0.0;

    // Create a virtual start waypoint at robot's current pose with a default method
    gps_waypoint start_wp(0.0, 0.0, "line");  // use "line" method for first segment
    start_wp.setMapPose(robot_pose_map);

    gps_waypoint prev_wp = start_wp;  // copy, not pointer

    for (auto& pair : remaining_waypoints) {
        gps_waypoint* wp_ptr = pair.first;
        const geometry_msgs::msg::Pose& map_pose = pair.second;

        // Set map pose into the waypoint for spline generation
        wp_ptr->setMapPose(map_pose);

        // Generate spline segment from prev_wp to current waypoint
        auto segment = gps_waypoint_spline::SplineFactory::generate(wp_ptr->method(), prev_wp, *wp_ptr);
        if (segment.empty()) {
            RCLCPP_WARN(this->get_logger(), "Spline generation failed, stopping chain.");
            break;
        }

        // Add segment poses to path
        for (const auto& pose : segment) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_map.header;
            ps.pose = pose;
            path_map.poses.push_back(ps);
        }

        // Calculate cumulative length
        for (size_t j = 1; j < segment.size(); ++j) {
            cumulative_length += std::hypot(segment[j].position.x - segment[j-1].position.x,
                                            segment[j].position.y - segment[j-1].position.y);
        }

        // Update previous waypoint for next iteration (copy)
        prev_wp = *wp_ptr;

        // Stop if we've reached the minimum required path length
        if (cumulative_length >= min_spline_length) {
            break;
        }
    }

    // Transform path to odom frame and publish if it works
    nav_msgs::msg::Path path_odom;
    try {
        auto transform = tf_buffer_.lookupTransform(odom_frame_id, map_frame_id, tf2::TimePointZero);
        for (const auto& pose_stamped : path_map.poses) {
            geometry_msgs::msg::PoseStamped ps_out;
            tf2::doTransform(pose_stamped, ps_out, transform);
            ps_out.header.frame_id = odom_frame_id;
            ps_out.header.stamp = path_map.header.stamp;
            path_odom.poses.push_back(ps_out);
        }
        path_odom.header.frame_id = odom_frame_id;
        path_odom.header.stamp = path_map.header.stamp;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "TF MAP->ODOM failed: %s", ex.what());
        return;
    }

    if (cumulative_length >= min_spline_length) {
        path_pub->publish(path_odom);
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GPS path too short (%.2f m)", cumulative_length);
    }
}
// gps_waypoint constructor implementation
gps_waypoint::gps_waypoint(double lon, double lat, const std::string& method, double radius)
    : longitude_(lon), latitude_(lat), method_(method), radius_(radius) {}

// Register node as a component
// todo chat why are we evening using this
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(yet_another_gps_publisher)