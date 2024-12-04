cmake_minimum_required(VERSION 3.5)
project(robot_dog)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Teleop executable
add_executable(teleop_turtle 
    src/TeleopTurtle.cpp
)

# Dependencies for teleop
ament_target_dependencies(teleop_turtle
    rclcpp
    geometry_msgs
)

# Install the executable
install(TARGETS 
    teleop_turtle
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()