cmake_minimum_required(VERSION 3.5)
project(robot_dog)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/Environment.cpp
    src/GameEnvironment.cpp
    src/main.cpp
)

# Header files
set(HEADERS
    include/Environment.h
    include/GameEnvironment.h
    include/Point.h
)

# Create the executable
add_executable(robot_dog ${SOURCES})

# Link libraries
ament_target_dependencies(robot_dog rclcpp turtlesim std_srvs)

# Install targets
install(TARGETS robot_dog
    DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/
    DESTINATION include)

# Ament package
ament_package()
