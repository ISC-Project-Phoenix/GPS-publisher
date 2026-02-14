#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include "gps_waypoint.hpp"

namespace gps_waypoint_spline {

using SplineGenerator = std::function<std::vector<geometry_msgs::msg::Pose>(
    const gps_waypoint& start,
    const gps_waypoint& end)>;

class SplineFactory
{
public:
    static void registerGenerator(const std::string& name, SplineGenerator gen);
    static SplineGenerator getGenerator(const std::string& name);
    static std::vector<geometry_msgs::msg::Pose> generate(
        const std::string& name,
        const gps_waypoint& start,
        const gps_waypoint& end);

private:
    static std::map<std::string, SplineGenerator>& registry();
};

}  // namespace gps_waypoint_spline