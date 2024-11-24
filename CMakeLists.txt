cmake_minimum_required(VERSION 3.8)
project(dog_robot)

# Compiler options for better warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

# Add the executable
add_executable(classroom_environment
  src/main.cpp       # Ensure main.cpp contains a 'main' function
  src/Environment.cpp
)

# Include directories
target_include_directories(environment_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link necessary dependencies
ament_target_dependencies(environment_test
  rclcpp
  turtlesim
)

# Install the executable
install(TARGETS environment_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
