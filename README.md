# Functionality

This node is designed to broadcast a spline between your Eagle and a waypoint(s). This is a ground truth full stack planning and navigation Node, desigened to be used with an Ackermann control setup downstream. This requires a MAP and ODOM frame and NavSat for transforms with a GNSS to robotspace. The algorythm takes in defined GPS waypoints and uses a spline generating function, which are created in a abstract factory spline_methods. This allows for different type of spline gen functions to be used with mininal downstream changes. Each point will specify a function type.  

# Parameters

- min_spline_length <double> This is just the minium length the spline needs to be. This is determined by what the Controls team needs. This is a hard coding trick for now but ideally this a topic from Hybird Pure Pursuit so it can be dynamic. 
- odom_topic <std::string> "odom_topic" 
- utm_frame_id <std::string>"utm_frame_id". Keep in mind this changes between Dearborn and Indina
- odom_frame_id <std::string> "odom_frame_id"
- waypoint_file_path <std::string> "waypoint_file" path
- max_gps_variance <double> so basically this is the amount of variance allowed in gps readings between gnss readings. Ideally we dont want to be running if the GPS is jumping between METERS of points compared to odom. 

# File structure

```
.
├── include
│   └── yet_another_gps_publisher
│       ├── gps_waypoint.hpp        // this is the header for the GPS CLASSES
│       ├── spline_factory.hpp      // this holds the spline generation methods
│       └── yet_another_gps_publisher_node.hpp  // this is the header for the node specficlly. 
├── package.xml // ros building files
├── README.md   // this file 😆 
├── src         // the main file for core logic
│   ├── spline_methods.cpp                  // this holds the spline generation methonds
│   ├── yet_another_gps_publisher.cpp       // this holds the main logic for the node. THis is where the callbacks are
│   └── yet_another_gps_publisher_node.cpp  // this is file that ROS launches. 
└── tests   // Placeholders for future unit testing 
    └── unit.cpp  
```

yet_another_gps_publisher_NODE_NAME_node: Source files for the ROS2 node object itself, and only itself

yet_another_gps_publisher_NODE_NAME.cpp: Source for the main function of the node, and only the main function

tests/unit.cpp: Example file for unit tests. This is linked to the node and ros, so both can be used
