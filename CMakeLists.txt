cmake_minimum_required(VERSION 3.8)
project(robot_dog)

# Enable C++17 and compiler warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Add a shared library for reusable environment code
add_library(environment_library SHARED
  src/Environment.cpp
  src/GameEnvironment.cpp
  src/CollisionHandler.cpp
)

# Link dependencies for the library
ament_target_dependencies(environment_library
  rclcpp
  turtlesim
)

# Add executables
add_executable(environment_building src/main_environment_builder.cpp)
add_executable(user_inputs src/UserTurtle.cpp)

# Link libraries and dependencies
target_link_libraries(environment_building environment_library)
target_link_libraries(user_inputs environment_library)

ament_target_dependencies(environment_building
  rclcpp
  turtlesim
)

ament_target_dependencies(user_inputs
  rclcpp
  turtlesim
  geometry_msgs
)

# Add test with working directory
ament_add_gtest(environment_builder_test 
  test/environment_builder_test.cpp
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
target_link_libraries(environment_builder_test environment_library)

# Install targets
install(TARGETS 
  environment_building 
  user_inputs
  environment_library
  DESTINATION lib/${PROJECT_NAME}
)

# Mark package as complete
ament_package()