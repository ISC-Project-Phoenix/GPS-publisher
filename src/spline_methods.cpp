#include <cmath>
#include <stdexcept>

#include "yet_another_gps_publisher/spline_factory.hpp"

namespace gps_waypoint_spline {

// Factory registry
std::map<std::string, SplineGenerator>& SplineFactory::registry() {
    static std::map<std::string, SplineGenerator> reg;
    return reg;
}

void SplineFactory::registerGenerator(const std::string& name, SplineGenerator gen) { registry()[name] = gen; }

SplineGenerator SplineFactory::getGenerator(const std::string& name) {
    auto it = registry().find(name);
    if (it == registry().end()) {
        throw std::runtime_error("Unknown spline method: " + name);
    }
    return it->second;
}

std::vector<geometry_msgs::msg::Pose> SplineFactory::generate(const std::string& name, const gps_waypoint& start,
                                                              const gps_waypoint& end) {
    return getGenerator(name)(start, end);
}

// ------------------------------------------------------------------
// Concrete generators
// ------------------------------------------------------------------

static std::vector<geometry_msgs::msg::Pose> linearGenerator(const gps_waypoint& start, const gps_waypoint& end) {
    const auto& a = start.odomPose().position;
    const auto& b = end.odomPose().position;

    const int num_points = 10;
    std::vector<geometry_msgs::msg::Pose> points;
    points.reserve(num_points + 1);

    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        geometry_msgs::msg::Pose p;
        p.position.x = a.x + t * (b.x - a.x);
        p.position.y = a.y + t * (b.y - a.y);
        p.position.z = a.z + t * (b.z - a.z);
        p.orientation.w = 1.0;
        points.push_back(p);
    }
    return points;
}

static std::vector<geometry_msgs::msg::Pose> circleGenerator(const gps_waypoint& start, const gps_waypoint& end) {
    double R = end.radius();
    if (R <= 0.0) {
        return linearGenerator(start, end);
    }

    const auto& a = start.odomPose().position;
    const auto& b = end.odomPose().position;

    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double chord = std::hypot(dx, dy);

    double theta = 2.0 * std::asin(chord / (2.0 * R));

    // TODO: Replace with actual circular arc interpolation.
    // For now, return linear as a fallback.
    return linearGenerator(start, end);
}

// Register generators (runs before main)
static bool registered = []() {
    SplineFactory::registerGenerator("linear", linearGenerator);
    SplineFactory::registerGenerator("circle", circleGenerator);
    return true;
}();

}  // namespace gps_waypoint_spline