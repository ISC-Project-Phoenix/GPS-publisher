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
    bool enabled() const { return enabled; }
    void setEnabled(bool e) { enabled_ = e; }

    // Odom pose after transformation (set when waypoint is loaded)
    void setOdomPose(const geometry_msgs::msg::Pose& pose) { odom_pose_ = pose; }
    const geometry_msgs::msg::Pose& odomPose() const { return odom_pose; }

private:
    double longitude_ = 0.0;
    double latitude_ = 0.0;
    std::string method_ = "linear";
    double radius_ = 0.0;
    bool enabled_ = true;
    geometry_msgs::msg::Pose odom_pose;
};