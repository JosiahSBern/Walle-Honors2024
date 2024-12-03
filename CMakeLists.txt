cmake_minimum_required(VERSION 3.5)
project(robot_dog)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add src as an include directory
include_directories(src)

# Source files
set(SOURCES
    src/Environment.cpp
    src/GameEnvironment.cpp
    src/TrashTurtle.cpp
    src/main.cpp
)

# Add executable
add_executable(robot_dog ${SOURCES})

# Link libraries and dependencies
ament_target_dependencies(robot_dog
  rclcpp
  turtlesim
  std_srvs
  geometry_msgs
)

# Install targets
install(TARGETS robot_dog
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers (even if in src)
install(FILES
    src/Environment.h
    src/GameEnvironment.h
    src/TrashTurtle.h
    src/Point.h
    DESTINATION include/${PROJECT_NAME}
)

ament_package()
