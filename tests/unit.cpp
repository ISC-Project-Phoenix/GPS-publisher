#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "yet_another_gps_publisher/yet_another_gps_publisher_node.hpp"

TEST(yet_another_gps_publisher, Test1) {}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}