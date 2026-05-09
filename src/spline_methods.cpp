#include <cmath>
#include <stdexcept>

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
void SplineFactory::registerGenerator(const std::string& name, SplineGenerator gen) { 
    registry()[name] = gen; 
}

/* getGenerator: inputs the name (classification of spline geometry), outputs the actual function maping in regsitry */
SplineGenerator SplineFactory::getGenerator(const std::string& name) {
    auto it = registry().find(name);
    if (it == registry().end()) {
        throw std::runtime_error("Unknown spline method: " + name);
    }
    return it->second;
}

/* generate: inputs the classification, waypoint W_0 and waypoint W_n, and ouptus the dynamic array of points*/
std::vector<geometry_msgs::msg::Pose> SplineFactory::generate(const std::string& name, const gps_waypoint& start, const gps_waypoint& end) {
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
    /*
    Chord = sqrt[(x_b - x_a)^2 + (y_b - y_a)^2]
    theta <- 2 * arcsin(chord/2R)

    */
    double R = end.radius();
    if (R <= 0.0) {
        return linearGenerator(start, end);
    }

    const auto& a = start.mapPose().position;
    const auto& b = end.mapPose().position;

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