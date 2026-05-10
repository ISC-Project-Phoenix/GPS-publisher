#pragma once

#include <geometry_msgs/msg/pose.hpp> /* for geoemtry_msg::msg::Pose for points */
#include <functional> /* for functional paradigme returing of a function */
#include <map> /* for map ADT {key: '', value: ''} pair */
#include <string> /* for method i.e 'linear', 'circular', etc.*/
#include <vector> /* for dynamic array ADT */

#include "gps_waypoint.hpp"

namespace gps_waypoint_spline {

// 'using' just creates a shorthand nickname. Instead of typing out
// this massive 'std::function<...>' every time, we can just type 'SplineGenerator'.
// when called write this!
// void registerGenerator(std::string name, SplineGenerator gen);
using SplineGenerator = std::function<std::vector<geometry_msgs::msg::Pose>(const gps_waypoint& start, const gps_waypoint& end)>;
/*
    input: waypoint start P_0, end P_n
    output: function that outputs vector of Poses P(x,y,z) vec = <P_0, P_1, P_n-1>
*/

class SplineFactory {
public:
    static void registerGenerator(const std::string& name, SplineGenerator gen); /* save function into memory */
    static SplineGenerator getGenerator(const std::string& name); /* looks up function based on key */
    static std::vector<geometry_msgs::msg::Pose> generate(const std::string& name, const gps_waypoint& start,
                                                          const gps_waypoint& end);  /* looks up the key and executes in one go */

private:
    static std::map<std::string, SplineGenerator>& registry(); /* {key: 'method', value: 'function()'} meyers singleton returing refrence to the map */
};

}  // namespace gps_waypoint_spline