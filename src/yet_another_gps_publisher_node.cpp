#include "yet_another_gps_publisher/yet_another_gps_publisher_node.hpp"

#include <chrono>
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

    // true for do GPS varance check false if not.
    do_gps_variance_check = this->declare_parameter<bool>("do_gps_variance_check", false);

    // This is the mimium size of the spline as required by the controls team. If its too short they cannot plan ahead of corners enough.
    min_spline_length = this->declare_parameter<double>("min_spline_length", 10.0);

    // this is the mimium radus for the kart to have considered "arrived" at a particular waypoint.
    // as is the norm for ROS2 this unit is in meters.
    arrival_threshold = this->declare_parameter<double>("arrival_threshold", 2.0);

    // why the odom topic is a parameter: in sim we use the filtered odometry from the sim, but on the real robot we might want to use a different topic or maybe even have it remapped from the sim topic to the real topic.
    odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/gps/filtered");
    // This is the utm Frame. Keep in might dearborn and purdue have different utm zones, so this might be necessary to change when we switch between the two or where ever you are.
    utm_frame_id = this->declare_parameter<std::string>("utm_frame_id", "utm");
    // This is the odom frame we will translate the spline into.
    odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
    // this is the MAP frame that we will store the waypoints in over time.
    map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
    // TODO actually set this parameter from launch file or command line, not hardcoded.
    // TODO indentify where this file should be stored?
    waypoint_file_path = this->declare_parameter<std::string>(
        "waypoint_file_path",
        "/home/isc/Documents/dev/phnx_ws_2026/src/gps_publisher/src/gps_waypoints_parking_lot_mk1.txt");

    // Publisher
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 5);

    // Subscribers
    
    // Subscribe to Raw GPS to check the fix status (VectorNav)
    raw_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/phoenix/navsat", 10, std::bind(&yet_another_gps_publisher::raw_gps_callback, this, std::placeholders::_1));

    // Subscribe to NavSat Transform output to trigger spline generation
    gps_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/navsat_gps", 10, std::bind(&yet_another_gps_publisher::global_ekf_callback, this, std::placeholders::_1));

    // Load waypoints and initialize iterator
    if (load_waypoints(waypoint_file_path)) {
        current_waypoint_it_ = waypoints.begin();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from file.", waypoints.size());
    } else {
        while (true){
            // FAIL LOUDLY
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints. Node will not publish paths.");
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from file.", waypoints.size());
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
        }
    }
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
    // int8 STATUS_NO_FIX =  -1        # unable to fix position
    // int8 STATUS_FIX =      0        # unaugmented fix
    // int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
    // int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
    // https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatStatus.html
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "GPS Lost Fix!");
        is_gps_valid = false;
        return;
    }

    if (max_gps_variance <= 0) {
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

    waypoints.clear();
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
    if ( line_num < 5 ) return false;
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

// Advance iterator
void yet_another_gps_publisher::advance_to_next_waypoint() {
    if (current_waypoint_it_ != waypoints.end()) {
        ++current_waypoint_it_;
        // wrap around for looping since the track is a circuit
        if (current_waypoint_it_ == waypoints.end()) {
            current_waypoint_it_ = waypoints.begin();
        }
    }
}

// generate spline path from robot until we exceed max spline
void yet_another_gps_publisher::global_ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Guard conditions
    if (!is_gps_valid || waypoints.empty()) {
        return;
    }

    geometry_msgs::msg::Pose robot_pose_map;
    try {
        geometry_msgs::msg::PoseStamped ps_in;
        ps_in.header = msg->header;  // frame_id from /odometry/gps (likely "odom")
        ps_in.pose = msg->pose.pose;
        auto ps_out = tf_buffer_.transform(ps_in, map_frame_id, std::chrono::milliseconds(100));
        robot_pose_map = ps_out.pose;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF /odometry/gps -> map failed: %s",
                             ex.what());
        return;
    }

    geometry_msgs::msg::Pose robot_postion = robot_pose_map;  //msg->pose.pose;

    size_t checked = 0;
    const size_t N = waypoints.size();

    while (checked < N) {
        // Transform the current waypoint to map frame on demand
        gps_waypoint& wp = *current_waypoint_it_;
        geometry_msgs::msg::Pose wp_map_pose;
        try {
            wp.utmPose().header.stamp = rclcpp::Time(0);
            geometry_msgs::msg::PoseStamped map_wp =
                tf_buffer_.transform(wp.utmPose(), map_frame_id, std::chrono::milliseconds(100));
            wp_map_pose = map_wp.pose;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF UTM->MAP failed: %s", ex.what());
            return;
        }

        double dist = std::hypot(robot_postion.position.x - wp_map_pose.position.x,
                                 robot_postion.position.y - wp_map_pose.position.y);
        if (dist >= arrival_threshold) break;  // not arrived yet do not skipping

        RCLCPP_INFO(this->get_logger(), "Passed waypoint (distance %.2f < %.2f)", dist, arrival_threshold);
        advance_to_next_waypoint();  // ++it, wraps to begin() if at end
        checked++;
    }

    if (checked >= N) {  // all waypoints reached → full lap done
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "All waypoints reached (full lap)");
        return;
    }

    nav_msgs::msg::Path path_map;
    path_map.header.frame_id = map_frame_id;
    path_map.header.stamp = msg->header.stamp;

    double cumulative_length = 0.0;

    // TODO decided if we want to start at the back of the robot pose
    gps_waypoint start_wp(0.0, 0.0, "line");
    start_wp.setMapPose(robot_postion);
    gps_waypoint prev_wp = start_wp;  // this will be updated as we drive forward

    // the look ahead scanner.
    auto segment_it = current_waypoint_it_;
    size_t processed = 0;
    const size_t n = waypoints.size();

    while (cumulative_length < min_spline_length) {
        gps_waypoint& wp = *segment_it;
	processed++;
	
        // Transform this waypoint to map frame
        geometry_msgs::msg::Pose map_pose;
        try {
            wp.utmPose().header.stamp = rclcpp::Time(0);
            geometry_msgs::msg::PoseStamped map_wp =
                tf_buffer_.transform(wp.utmPose(), map_frame_id, std::chrono::milliseconds(100));
            map_pose = map_wp.pose;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF transform failed during spline build: %s", ex.what());
            break;
        }
        wp.setMapPose(map_pose);

        // Generate spline segment between prev_wp and wp
        auto segment = gps_waypoint_spline::SplineFactory::generate(wp.method(), prev_wp, wp);
        if (segment.empty()) {
            RCLCPP_WARN(this->get_logger(), "Spline generation failed, stopping chain.");
            break;
        }

        // Add segment poses to the path
        for (const auto& pose : segment) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_map.header;
            ps.pose = pose;
            path_map.poses.push_back(ps);
        }

        // Calculate cumulative length
        for (size_t j = 1; j < segment.size(); ++j) {
            cumulative_length += std::abs( std::hypot(segment[j].position.x - segment[j - 1].position.x,
                                            segment[j].position.y - segment[j - 1].position.y) );
        }

        prev_wp = wp;  // shift anchor
        ++segment_it;  // advance the scanning pointer
        processed++;

        // to stop if we’ve walked the whole list without hitting the length.
        // TODO decide is we still wnat to publish?
        // should probably warn though. I dont see this ever being a problem though.
        if (processed >= waypoints.size()) break;
    }

    // Transform to odom and publish
    nav_msgs::msg::Path path_odom;
    try {
        auto transform = tf_buffer_.lookupTransform(odom_frame_id, map_frame_id, tf2::TimePointZero);
        for (const auto& ps : path_map.poses) {
            geometry_msgs::msg::PoseStamped ps_out;
            tf2::doTransform(ps, ps_out, transform);
            ps_out.header.frame_id = odom_frame_id;
            ps_out.header.stamp = path_map.header.stamp;
            path_odom.poses.push_back(ps_out);
        }
        path_odom.header.frame_id = odom_frame_id;
        path_odom.header.stamp = path_map.header.stamp;
    } catch (tf2::TransformException& ex) {
        // apparently the map look up has failed!
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 0, "TF MAP->ODOM failed: %s", ex.what());
        return;
    }

    if (cumulative_length >= min_spline_length) {
        path_pub->publish(path_odom);
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 0, "GPS path too short (%.2f m)",
                             (double)cumulative_length);
    }
}

// gps_waypoint constructor implementation
gps_waypoint::gps_waypoint(double lon, double lat, const std::string& method, double radius)
    : longitude_(lon), latitude_(lat), method_(method), radius_(radius) {}

// Register node as a component
// todo chat why are we evening using this here
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(yet_another_gps_publisher)
