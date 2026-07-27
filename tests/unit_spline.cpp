#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "yet_another_gps_publisher/gps_waypoint.hpp"
#include "yet_another_gps_publisher/spline_factory.hpp"

/* put your own here or fix the CMAKE I am not going to do it. */
#define WAYPOINT_FILE_TEST "/home/elijahstickel/Desktop/phnx_dev/src/gps_publisher/data/example_waypoints_circle.txt"
// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static gps_waypoint make_waypoint(double x, double y, const std::string& method, double radius = 0.0) {
    gps_waypoint wp(0.0, 0.0, method, radius);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    wp.setMapPose(pose);
    return wp;
}

// ---------------------------------------------------------------------------
// GPS file test — mirrors your load_waypoints logic but minimal
// ---------------------------------------------------------------------------

TEST(yet_another_gps_publisher, gps_file_loads_correctly) {
    const std::string path = std::string(WAYPOINT_FILE_TEST);
    std::ifstream file(path);
    ASSERT_TRUE(file.is_open()) << "Could not open: " << path;

    int count = 0;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        double lon, lat;
        std::string method;
        ASSERT_TRUE(iss >> lon >> lat >> method) << "Malformed line: " << line;
        count++;
    }
    EXPECT_GT(count, 0) << "Waypoint file is empty";
    RCLCPP_INFO(rclcpp::get_logger("test"), "Loaded %d waypoints from file", count);
}

// ---------------------------------------------------------------------------
// Circle generator — output to CSV for Desmos
// ---------------------------------------------------------------------------

TEST(yet_another_gps_publisher, circle_generator_desmos_output) {
    // Two points ~7m apart, 5m radius left turn
    gps_waypoint start = make_waypoint(0.0, 0.0, "linear");
    gps_waypoint end = make_waypoint(5.0, 5.0, "circle", 5.0);

    auto points = gps_waypoint_spline::SplineFactory::generate("circle", start, end);

    ASSERT_GT(points.size(), 0u) << "Circle generator returned no points";

    // Dump to CSV so you can paste into Desmos
    const std::string csv_path = "/tmp/circle_arc_output.csv";
    std::ofstream csv(csv_path);
    ASSERT_TRUE(csv.is_open()) << "Could not open output CSV";
    csv << "x,y\n";
    for (auto& p : points) {
        csv << p.position.x << "," << p.position.y << "\n";
    }
    csv.close();
    RCLCPP_INFO(rclcpp::get_logger("test"), "Arc points written to %s — paste into Desmos table", csv_path.c_str());

    // Basic sanity: all points should be within radius*2 of start
    for (auto& p : points) {
        double dist = std::hypot(p.position.x, p.position.y);
        EXPECT_LT(dist, 20.0) << "Point suspiciously far from origin";
    }
}

// ---------------------------------------------------------------------------
// Circle fallback — bad radius should return linear points
// ---------------------------------------------------------------------------

TEST(yet_another_gps_publisher, circle_generator_falls_back_on_bad_radius) {
    gps_waypoint start = make_waypoint(0.0, 0.0, "linear");
    gps_waypoint end = make_waypoint(10.0, 0.0, "circle", 0.0);  // zero radius

    auto points = gps_waypoint_spline::SplineFactory::generate("circle", start, end);
    ASSERT_GT(points.size(), 0u);

    // All points should be on y=0 (linear fallback)
    for (auto& p : points) {
        EXPECT_NEAR(p.position.y, 0.0, 1e-6) << "Expected linear fallback on y=0";
    }
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);
    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}