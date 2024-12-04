cmake_minimum_required(VERSION 3.8)
project(robot_dog)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Teleop executable
add_executable(teleop_turtle 
    src/TeleopTurtle.cpp
)

# Link dependencies
ament_target_dependencies(teleop_turtle
    rclcpp
    geometry_msgs
)

# Install the executable
install(TARGETS 
    teleop_turtle
    DESTINATION lib/${PROJECT_NAME}
)

# Necessary for a ROS2 package
ament_package()