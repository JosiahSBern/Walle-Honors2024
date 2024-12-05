cmake_minimum_required(VERSION 3.5)
project(robot_dog)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add src as an include directory
include_directories(src)

# Main executable source files
set(SOURCES
  src/Environment.cpp
  src/GameEnvironment.cpp
  src/TeleopTurtle.cpp
  src/Turtle.cpp
  src/TrashTurtle.cpp
  src/main.cpp
)

# Add the main executable for the game environment
add_executable(robot_dog ${SOURCES})

# Add a separate executable for teleop control (if truly independent)
add_executable(teleop_turtle src/TeleopTurtle.cpp)

# Link libraries and dependencies for robot_dog
ament_target_dependencies(robot_dog
  rclcpp
  turtlesim
  geometry_msgs
)

# Link libraries and dependencies for teleop_turtle
ament_target_dependencies(teleop_turtle
  rclcpp
  geometry_msgs
)

# Install the executables
install(TARGETS 
  robot_dog
  teleop_turtle
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (assuming all headers are in the src directory)
install(DIRECTORY src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
)

ament_package()
