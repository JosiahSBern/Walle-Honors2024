cmake_minimum_required(VERSION 3.8)
project(robot_dog)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

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


# Install executables
install(TARGETS 
  environment_building 
  user_inputs
  DESTINATION lib/${PROJECT_NAME}
)

# Mark package as complete
ament_package()
