#include "yet_another_gps_publisher/yet_another_gps_publisher_node.hpp"

#include <cmath>
#include <fstream>
#include <geographic_msgs/msg/geo_point.hpp>
#include <memory>
#include <sstream>

#include "yet_another_gps_publisher/spline_factory.hpp"

// Constructor
yet_another_gps_publisher::yet_another_gps_publisher(const rclcpp::NodeOptions& options)
    : Node("yet_another_gps_publisher", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Declare parameters

    // Threshold for GPS "Confidence". 0.1 means we only trust the GPS if it's within ~30cm precision.
    // TODO figure out what a good threshold is based on the actual GPS variance we see in testing, and maybe even make it adaptive based on current conditions.
    max_gps_variance = this->declare_parameter<double>("max_gps_variance", 0.1);

    // This is the mimium size of the spline as required by the controls team. If its too short they cannot plan ahead of corners enough.
    min_spline_length = this->declare_parameter<double>("min_spline_length", 10.0);

    // why the odom topic is a parameter: in sim we use the filtered odometry from the sim, but on the real robot we might want to use a different topic or maybe even have it remapped from the sim topic to the real topic.
    odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    // This is the utm Frame. Keep in might dearborn and purdue have different utm zones, so this might be necessary to change when we switch between the two or where ever you are.
    utm_frame_id = this->declare_parameter<std::string>("utm_frame_id", "utm");
    // This is the odom frame we will translate the waypoints to.
    odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
    // TODO actually set this parameter from launch file or command line, not hardcoded.
    // TODO indentify where this file should be stored?
    waypoint_file_path = this->declare_parameter<std::string>("src", "gps_waypoints_parking_lot_mk1.txt");

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

    // Load waypoints directly on startup!
    // TODO catch failure and maybe retry later if file not found, instead of just crashing or doing nothing.
    load_waypoints(waypoint_file_path);
}

// Odom callback
void yet_another_gps_publisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose = msg->pose.pose;
}

// The Confidence Check + RAW GPS callback
void yet_another_gps_publisher::raw_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
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

// standard function to load waypoints from file on startup
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

        waypoints.push_back(wp);
        RCLCPP_INFO(this->get_logger(), "Loaded waypoint %zu: spline_type=%s at (%.6f, %.6f)", waypoints.size(),
                    spline_type.c_str(), lon, lat);
    }
    file.close();
    return true;
}

// Transform waypoint from lat/lon to odom
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

// gps callback: generate and publish spline
void yet_another_gps_publisher::gps_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Guard: Don't calculate paths if waypoints aren't loaded or GPS is unreliable
    if (!is_gps_valid || waypoints.empty()) {
        return;
    }

    // Update current robot pose from the GPS-corrected odometry message
    current_pose = msg->pose.pose;

    // --- STEP 1: DYNAMIC TRANSFORM ---
    // We transform our static UTM waypoints into the current (drifting) ODOM frame
    for (auto& wp : waypoints) {
        try {
            // Use time 0 to get the latest available transform
            wp.utmPose().header.stamp = rclcpp::Time(0);
            geometry_msgs::msg::PoseStamped odom_wp = tf_buffer_.transform(wp.utmPose(), odom_frame_id);
            wp.setOdomPose(odom_wp.pose);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Link UTM->ODOM failed: %s",
                                  ex.what());
            return;
        }
    }

    // --- STEP 2: PATH GENERATION ---
    nav_msgs::msg::Path path;
    path.header.frame_id = odom_frame_id;
    path.header.stamp = msg->header.stamp;  // Sync path time to the GPS update time

    double cumulative_length = 0.0;
    size_t used_count = 0;

    // Start the spline from the robot's current position
    gps_waypoint current_wp;
    current_wp.setOdomPose(current_pose);

    const gps_waypoint* start_ptr = &current_wp;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto segment = gps_waypoint_spline::SplineFactory::generate(waypoints[i].method(), *start_ptr, waypoints[i]);
        start_ptr = &waypoints[i];

        for (const auto& pose : segment) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose = pose;
            path.poses.push_back(ps);
        }

        // Calculate length of this segment to see if we've met the min_spline_length requirement
        for (size_t j = 1; j < segment.size(); ++j) {
            cumulative_length += std::hypot(segment[j].position.x - segment[j - 1].position.x,
                                            segment[j].position.y - segment[j - 1].position.y);
        }

        used_count = i + 1;
        if (cumulative_length >= min_spline_length) break;
    }

    // --- STEP 3: PUBLISH ---
    if (cumulative_length >= min_spline_length) {
        path_pub->publish(path);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "GPS path too short (%.2f m), waiting for more waypoints", cumulative_length);
    }
}
// gps_waypoint constructor implementation
gps_waypoint::gps_waypoint(double lon, double lat, const std::string& method, double radius)
    : longitude_(lon), latitude_(lat), method_(method), radius_(radius) {}

// Register node as a component
// todo chat why are we evening using this
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(yet_another_gps_publisher)