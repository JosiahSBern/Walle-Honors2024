cmake_minimum_required(VERSION 3.8)
project(robot_dog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

# Add executable
add_executable(robot_dog 
  src/Environment.cpp 
  src/main.cpp
)

# Link libraries
ament_target_dependencies(robot_dog 
  rclcpp 
  turtlesim
)

# Install targets
install(TARGETS robot_dog
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()