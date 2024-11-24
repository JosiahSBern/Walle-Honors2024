cmake_minimum_required(VERSION 3.8)
project(dog_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Add executable
add_executable(classroom_node 
  src/main.cpp
  src/Environment.cpp
)

ament_target_dependencies(classroom_node
  rclcpp
  turtlesim
)

install(TARGETS
  classroom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()
