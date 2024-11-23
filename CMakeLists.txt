cmake_minimum_required(VERSION 3.8)
project(dog_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

# Add executable
add_executable(environment_test
  src/main.cpp
  src/Environment.cpp
)

# Include directories
target_include_directories(environment_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(environment_test
  rclcpp
  turtlesim
)

# Install
install(TARGETS environment_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
