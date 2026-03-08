#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <string>

class gps_waypoint {
public:
    gps_waypoint() = default;
    gps_waypoint(double lon, double lat, const std::string& method, double radius = 0.0);

    double longitude() const { return longitude; }
    double latitude() const { return latitude; }
    const std::string& method() const { return method; }
    double radius() const { return radius; }

    // Odom pose after transformation (set when waypoint is loaded)
    // TODO if we are still doing ODOM frame for storing maps or just as temp frame for controls
    void setOdomPose(const geometry_msgs::msg::Pose& pose) { odom_pose_ = pose; }
    const geometry_msgs::msg::Pose& odomPose() const { return odom_pose; }

private:
    double longitude = 0.0;
    double latitude = 0.0;
    std::string method = "linear";
    double radius = 0.0;
    geometry_msgs::msg::Pose odom_pose;
};