#include "Environment.h"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>  
using namespace std;

// Constants for TurtleSim window
const double TURTLESIM_WINDOW_WIDTH = 11.0;
const double TURTLESIM_WINDOW_HEIGHT = 11.0;

ClassroomEnvironment::ClassroomEnvironment(rclcpp::Node::SharedPtr node)
    : Environment(node) {
    // Calculate room dimensions based on TurtleSim window
    roomWidth = TURTLESIM_WINDOW_WIDTH - 1.0;  // Leave 0.5 margin on each side
    roomLength = TURTLESIM_WINDOW_HEIGHT - 1.0;
    
    // Calculate desk dimensions to fill the room evenly
    desk_spacing = 0.5;  // Consistent spacing
    desk_width = (roomWidth - (desksPerRow + 1) * desk_spacing) / desksPerRow;
    desk_height = (roomLength - (desksPerColumn + 1) * desk_spacing) / desksPerColumn;
} 
ClassroomEnvironment::~ClassroomEnvironment() {
}

void ClassroomEnvironment::drawWalls() {
    // Define the corners of the classroom precisely
    Point bottomLeft = {0.5, 0.5};
    Point bottomRight = {roomWidth + 0.5, 0.5};
    Point topLeft = {0.5, roomLength + 0.5};
    Point topRight = {roomWidth + 0.5, roomLength + 0.5};

    // Set pen color to black for walls
    setPen(true, 0, 0, 0, 3);  // Thicker wall line

    // Draw the classroom walls
    drawLine(bottomLeft, bottomRight);  // Bottom wall
    drawLine(bottomRight, topRight);    // Right wall
    drawLine(topRight, topLeft);        // Top wall
    drawLine(topLeft, bottomLeft);      // Left wall
}

void ClassroomEnvironment::drawExit() {
    // Make sure to wait for the pen service to be available
    while (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for pen service");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for pen service...");
    }

    // Set the pen color to green with full opacity
    setPen(true, 0, 255, 0, 2);
      
    Point exitPosition = {roomWidth + 0.5, roomLength + 0.5};
    drawLine(exitPosition, {exitPosition.x + 0.5, exitPosition.y});
    
    // Set the exit coordinates
    setExit(exitPosition.x + 0.5, exitPosition.y, "north");
    RCLCPP_INFO(node_->get_logger(), "Exit position set at: (%f, %f)", exitPosition.x + 0.5, exitPosition.y);
}

void ClassroomEnvironment::drawDesk() {
    // Ensure pen service is available
    while (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for pen service");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for pen service...");
    }

    // Set the pen color to brown
    setPen(true, 139, 69, 19, 2);
    
    int desksPerRow = 3;
    int desksPerColumn = 3;

    for (int row = 0; row < desksPerColumn; ++row) {
        for (int col = 0; col < desksPerRow; ++col) {
            // Calculate x and y positions with even spacing
            double x_position = 0.5 + desk_spacing + col * (desk_width + desk_spacing);
            double y_position = 0.5 + desk_spacing + row * (desk_height + desk_spacing);
            
            // Draw the desk as a rectangle
            Point topLeft = {x_position, y_position};
            Point bottomRight = {topLeft.x + desk_width, topLeft.y + desk_height};
            drawRectangle(topLeft, bottomRight);
        }
    }
}

// void ClassroomEnvironment::spawnStartingTurtle() {
//     // Create spawn client if not already created
//     if (!spawn_client_) {
//         spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");
//     }

//     auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    
//     // Position the new turtle near the bottom-right corner
//     request->x = roomWidth;  
//     request->y = 1.0;        
//     request->theta = M_PI / 2;  // Facing upward (90 degrees)
//     request->name = "turtle2";  

//     // Send the spawn request
//     auto result = spawn_client_->async_send_request(request);
    
//     if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_INFO(node_->get_logger(), "Successfully spawned turtle2 at starting position.");
//     } else {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle2.");
//     }
// }

void ClassroomEnvironment::drawClassroom(){
    drawWalls();
    drawExit();
    drawDesk();
    // spawnStartingTurtle();
    quit();
}

// Modify setPen to allow line width specification
