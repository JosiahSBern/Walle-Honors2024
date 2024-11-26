#include <rclcpp/rclcpp.hpp>
#include "Environment.h"

#include <rclcpp/rclcpp.hpp>
#include "Environment.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_dog_node");

    // Create classroom environment
    ClassroomEnvironment classroom(node);
    
    // Draw the classroom
    classroom.drawClassroom();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}