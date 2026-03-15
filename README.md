An opinionated ROS2 C++ node template, optimised for ISC.

# Dependencies
Some common extra dependencies are included. Review them and remove what you don't need.
These are marked with yet_another_gps_publisher.

# Features

- Unit tests
- ROS-Industrial github CI (will test units and lints)
- C++ formatting via clangformat
- A selection of sane lints
- A single node setup in a multithreaded executor

# Parameters

- min_spline_length <double> This is just the minium length the spline needs to be. This is Determened by what the Controls team needs. this is a hard coding trick for now but ideally this a topic from Hybird Pure Pursuit so it can be dymatic. 
- odom_topic <std::string> "odom_topic" 
- utm_frame_id <std::string>"utm_frame_id". Keep in might this changes between Dearborn and Indina
- odom_frame_id <std::string> "odom_frame_id"
- waypoint_file_path <std::string> "waypoint_file" path

# File structure

```
.
├── include
│   └── yet_another_gps_publisher
│       ├── gps_waypoint.hpp        // this is the header for the GPS CLASSES
│       ├── spline_factory.hpp      // this holds the spline generation methonds
│       └── yet_another_gps_publisher_node.hpp  // this is the header for teh node specficlly. 
├── package.xml // ros building files
├── README.md   // this file 😆 
├── src         // the main file for logicing codes
│   ├── spline_methods.cpp                  // this holds the spline generation methonds
│   ├── yet_another_gps_publisher.cpp       // this holds the main logic for the node. THis is where the callbacks are
│   └── yet_another_gps_publisher_node.cpp  // this is file that ROS launches. 
└── tests   // we donts use these tbh
    └── unit.cpp  
```

yet_another_gps_publisher_NODE_NAME_node: Source files for the ROS2 node object itself, and only itself

yet_another_gps_publisher_NODE_NAME.cpp: Source for the main function of the node, and only the main function

tests/unit.cpp: Example file for unit tests. This is linked to the node and ros, so both can be used