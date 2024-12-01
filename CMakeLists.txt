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

# Add a shared library for reusable environment code
add_library(environment_library
  src/Environment.cpp
  src/GameEnvironment.cpp
  src/CollisionHandler.cpp
)

# Link dependencies for the library
ament_target_dependencies(environment_library
  rclcpp
  turtlesim
)

# Add executable for Environment Building
add_executable(environment_building
  src/main.cpp  # Just use a single main file
)

# Link the library to the Environment Building executable
target_link_libraries(environment_building environment_library)

ament_target_dependencies(environment_building
  rclcpp
  turtlesim
  std_srvs
)

# Install executables and library
install(TARGETS 
  environment_building 
  environment_library
  DESTINATION lib/${PROJECT_NAME}
)

# Mark package as complete
ament_package()