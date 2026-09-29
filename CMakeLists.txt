cmake_minimum_required(VERSION 3.8)
project(yet_another_gps_publisher)
include(FetchContent)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Messages TODO_EXTRA
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# GPS packages
find_package(geodesy REQUIRED)

# Add source for node executable (link non-ros dependencies here)
add_executable(yet_another_gps_publisher src/yet_another_gps_publisher.cpp src/yet_another_gps_publisher_node.cpp)
target_include_directories(yet_another_gps_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(yet_another_gps_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make ros deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        # Messages TODO_EXTRA
        ackermann_msgs
        sensor_msgs
        std_msgs
        # GPS Packages
        geodesy
        )

# Link ros dependencies
ament_target_dependencies(
        yet_another_gps_publisher
        ${dependencies}
)

install(TARGETS yet_another_gps_publisher
        DESTINATION lib/${PROJECT_NAME})

# Uncomment below to make launch files available if created
#install(
#        DIRECTORY launch config
#        DESTINATION share/${PROJECT_NAME}/
#)

if (BUILD_TESTING)
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            # Remember to add node source files
            src/yet_another_gps_publisher_node.cpp
            )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
endif ()

ament_package()
