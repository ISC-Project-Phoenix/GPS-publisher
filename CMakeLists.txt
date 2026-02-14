cmake_minimum_required(VERSION 3.8)
project(yet_another_gps_publisher)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Optional: print found packages for debugging
message(STATUS "Found tf2: ${tf2_DIR}")
message(STATUS "Found tf2_ros: ${tf2_ros_DIR}")
message(STATUS "Found tf2_geometry_msgs: ${tf2_geometry_msgs_DIR}")

# Add source for node executable
add_executable(yet_another_gps_publisher 
    src/yet_another_gps_publisher.cpp 
    src/yet_another_gps_publisher_node.cpp
    src/spline_methods.cpp
)
target_include_directories(yet_another_gps_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(yet_another_gps_publisher PUBLIC c_std_99 cxx_std_17)

set(dependencies
        rclcpp
        ackermann_msgs
        sensor_msgs
        std_msgs
        geodesy
        geographic_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        rclcpp_components
)

ament_target_dependencies(yet_another_gps_publisher ${dependencies})

install(TARGETS yet_another_gps_publisher
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            src/yet_another_gps_publisher_node.cpp
            src/spline_methods.cpp
            )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
endif ()

ament_package()