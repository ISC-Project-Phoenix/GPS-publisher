#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

class gps_waypoint {
public:
    gps_waypoint() = default;
    gps_waypoint(double lon, double lat, const std::string& method, double radius = 0.0);

    double longitude() const { return longitude_; }
    double latitude() const { return latitude_; }
    const std::string& method() const { return method_; }
    double radius() const { return radius_; }

    // Odom pose after transformation (set when waypoint is loaded)
    // TODO if we are still doing ODOM frame for storing maps or just as temp frame for controls
    void setOdomPose(const geometry_msgs::msg::Pose& pose) { odom_pose = pose; }
    const geometry_msgs::msg::Pose& odomPose() const { return odom_pose; }

    // Store the absolute UTM pose
    void setUtmPose(const geometry_msgs::msg::PoseStamped& pose) { utm_pose_ = pose; }
    geometry_msgs::msg::PoseStamped& utmPose() { return utm_pose_; }

    // Store the map-frame pose (for path generation)
    void setMapPose(const geometry_msgs::msg::Pose& pose) { map_pose_ = pose; }
    const geometry_msgs::msg::Pose& mapPose() const { return map_pose_; }

private:
    double longitude_ = 0.0;
    double latitude_ = 0.0;
    std::string method_ = "linear";
    double radius_ = 0.0;
    geometry_msgs::msg::PoseStamped utm_pose_;
    geometry_msgs::msg::Pose odom_pose;
    geometry_msgs::msg::Pose map_pose_;
};