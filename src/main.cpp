#include <rclcpp/rclcpp.hpp>
#include "Environment.h"

int main(int argc, char** argv) {
        // Initialize ROS2
        rclcpp::init(argc, argv);
    
        // Create node
        auto node = std::make_shared<rclcpp::Node>("classroom_environment");
        // Create rectangular room
        ClassroomEnvironment classroom(node);
        
        // Draw the classroom
        classroom.drawClassroom();

        
        // Keep the node running
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
}