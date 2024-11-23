#include <rclcpp/rclcpp.hpp>
#include "Environment.h"

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create node
    auto node = std::make_shared<rclcpp::Node>("environment_test");
        // Create rectangular room
        RectangularRoom room(node);
        
        // Set exit at the top (north)
        room.setExit(5.0, 10.0, "north");
        
        // Draw the room
        room.drawWalls();
        
        // Keep the node running
        rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}