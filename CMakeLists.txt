cmake_minimum_required(VERSION 3.5)
project(robot_dog)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add src as an include directory
include_directories(src)

# Source files
set(SOURCES
    src/Environment.cpp
    src/GameEnvironment.cpp
    src/Turtle.cpp
    src/TrashTurtle.cpp
    src/main.cpp
)

# Main executable
add_executable(robot_dog ${SOURCES})
add_executable(user_turtle src/UserTurtle.cpp)

# Link libraries and dependencies for robot_dog
ament_target_dependencies(robot_dog
  rclcpp
  turtlesim
  geometry_msgs
)


# Link libraries and dependencies for UserTurtle
ament_target_dependencies(user_turtle
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS 
    robot_dog
    user_turtle
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers (even if in src)
install(FILES
    src/Environment.h
    src/GameEnvironment.h
    src/TrashTurtle.h
    src/Point.h
    src/Turtle.h
    src/UserTurtle.cpp
    DESTINATION include/${PROJECT_NAME}
)

ament_package()
