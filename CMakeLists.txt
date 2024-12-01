cmake_minimum_required(VERSION 3.5)
project(robot_dog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(src)

set(SOURCES
    src/Environment.cpp
    src/GameEnvironment.cpp
    src/main.cpp
)

add_executable(robot_dog ${SOURCES})

ament_target_dependencies(robot_dog rclcpp turtlesim std_srvs)

install(TARGETS robot_dog
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    src/Environment.h
    src/GameEnvironment.h
    src/Point.h
    DESTINATION include/robot_dog
)

ament_package()
