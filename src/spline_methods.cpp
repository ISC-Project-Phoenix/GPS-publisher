#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "yet_another_gps_publisher/spline_factory.hpp"

// ------------------------------------------------------------------------------------------
//
//  todo this file is for storing the splines functions we want to generate with
//  we need to store the functions either here or in the GPS classes.
//  up too ya'll
//
// ------------------------------------------------------------------------------------------

namespace gps_waypoint_spline {

// Factory registry
std::map<std::string, SplineGenerator>& SplineFactory::registry() {
    static std::map<std::string, SplineGenerator> reg;
    return reg;
}

/* registerGenerator: inputs name (classification of spline geometry), and function of spline geometry stores them in regsitry {key: '', value: ''}*/
void SplineFactory::registerGenerator(const std::string& name, SplineGenerator gen) { registry()[name] = gen; }

/* getGenerator: inputs the name (classification of spline geometry), outputs the actual function maping in regsitry */
SplineGenerator SplineFactory::getGenerator(const std::string& name) {
    auto it = registry().find(name);
    if (it == registry().end()) {
        throw std::runtime_error("Unknown spline method: " + name);
    }
    return it->second;
}

/* generate: inputs the classification, waypoint W_0 and waypoint W_n, and ouptus the dynamic array of points*/
std::vector<geometry_msgs::msg::Pose> SplineFactory::generate(const std::string& name, const gps_waypoint& start,
                                                              const gps_waypoint& end) {
    return getGenerator(name)(start, end);
}

// ------------------------------------------------------------------
// Concrete generators
// ------------------------------------------------------------------

static std::vector<geometry_msgs::msg::Pose> linearGenerator(const gps_waypoint& start, const gps_waypoint& end) {
    /*
    a <- start position
    b <- end position 
    t <- [0, 1]
    P(t) = a + t(b - a)
    */
    const auto& a = start.mapPose().position;
    const auto& b = end.mapPose().position;

    const int num_points = 10;  // TODO this shouldnt be hardcoded like EVER
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
    double R_val = end.radius();

    if (std::abs(R_val) <= 0.001) {
        return linearGenerator(start, end);
    }

    const auto& a = start.mapPose().position;
    const auto& b = end.mapPose().position;

    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double chord = std::hypot(dx, dy);

    if (std::abs(R_val) < chord / 2.0) {
        return linearGenerator(start, end);
    }

    double d = std::sqrt(R_val * R_val - (chord / 2.0) * (chord / 2.0));
    double mx = (a.x + b.x) / 2.0;
    double my = (a.y + b.y) / 2.0;
    double side = (R_val > 0) ? 1.0 : -1.0;
    double cx = mx - side * d * (dy / chord);
    double cy = my + side * d * (dx / chord);

    double angle_start = std::atan2(a.y - cy, a.x - cx);
    double angle_end = std::atan2(b.y - cy, b.x - cx);

    double diff = angle_end - angle_start;
    if (side > 0) {
        while (diff <= 0.0) diff += 2.0 * M_PI;
        while (diff > 2.0 * M_PI) diff -= 2.0 * M_PI;
    } else {
        while (diff >= 0.0) diff -= 2.0 * M_PI;
        while (diff < -2.0 * M_PI) diff += 2.0 * M_PI;
    }

    double arc_length = std::abs(R_val * diff);
    int num_points = std::max(2, static_cast<int>(arc_length / 0.1));

    std::vector<geometry_msgs::msg::Pose> points;
    points.reserve(num_points + 1);

    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double current_angle = angle_start + t * diff;

        geometry_msgs::msg::Pose p;
        p.position.x = cx + std::abs(R_val) * std::cos(current_angle);
        p.position.y = cy + std::abs(R_val) * std::sin(current_angle);
        p.position.z = a.z + t * (b.z - a.z);

        double tangent_angle = current_angle + std::copysign(M_PI / 2.0, diff);

        tf2::Quaternion q;
        q.setRPY(0, 0, tangent_angle);
        p.orientation = tf2::toMsg(q);

        points.push_back(p);
    }
    return points;
}

// Register generators (runs before main)
static bool registered = []() {
    SplineFactory::registerGenerator("linear", linearGenerator);
    SplineFactory::registerGenerator("circle", circleGenerator);
    return true;
}();

}  // namespace gps_waypoint_spline