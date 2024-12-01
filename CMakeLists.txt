cmake_minimum_required(VERSION 3.8)
project(robot_dog)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable for Environment Building
add_executable(environment_building
  src/Environment.cpp
  src/GameEnvironment.cpp
  src/CollisionHandler.cpp
)

# Link libraries for Environment Building
ament_target_dependencies(environment_building
  rclcpp
  turtlesim
)

# Add executable for User Inputs
add_executable(user_inputs
  src/UserTurtle.cpp
)

# Link libraries for User Inputs
ament_target_dependencies(user_inputs
  rclcpp
  turtlesim
  geometry_msgs
)

# Add test executable for Environment Builder Testing
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(environment_builder_test test/environment_builder_test.cpp)
target_link_libraries(environment_builder_test environment_building)

# Link dependencies for tests
ament_target_dependencies(environment_builder_test
  rclcpp
  turtlesim
)

# Install executables
install(TARGETS 
  environment_building 
  user_inputs
  environment_builder_test
  DESTINATION lib/${PROJECT_NAME}
)

# Mark package as complete
ament_package()
