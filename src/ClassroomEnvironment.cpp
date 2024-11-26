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
    : Environment(node),
      desksPerRow(3),  // Example: 3 desks per row
      desksPerColumn(3) // Example: 3 desks per column
{
    roomWidth = TURTLESIM_WINDOW_WIDTH - 1.0;
    roomLength = TURTLESIM_WINDOW_HEIGHT - 1.0;
    
    desk_spacing = 0.5;
    // Fixed: separate calculation lines, added semicolons
    desk_width = (roomWidth - (desksPerRow + 1) * desk_spacing) / (desksPerRow * 2);
    desk_height = (roomLength - (desksPerColumn + 1) * desk_spacing) / (desksPerColumn * 2);
}

void ClassroomEnvironment::drawWalls() {
    // Draw white background first
    setPen(true, 255, 255, 255, 20);  // White pen
    
    // Cover the entire TurtleSim window with a white rectangle
    Point backgroundTopLeft = {0, 0};
    Point backgroundBottomRight = {TURTLESIM_WINDOW_WIDTH, TURTLESIM_WINDOW_HEIGHT};
    drawRectangle(backgroundTopLeft, backgroundBottomRight);

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

    // Set the pen color to brown (using magenta as an example)
    setPen(true, 255, 0, 255, 2);
    
    for (int row = 0; row < desksPerColumn; ++row) {
        for (int col = 0; col < desksPerRow; ++col) {
            // Calculate x and y positions with even spacing
            // Fixed: added missing semicolon, corrected syntax
            double x_position = 0.5 + desk_spacing + col * (desk_width * 2 + desk_spacing);
            double y_position = 0.5 + desk_spacing + row * (desk_height * 2 + desk_spacing);
            
            // Draw the desk as a rectangle
            // Fixed: created explicit Point objects
            Point topLeft = {x_position, y_position};
            Point bottomRight = {x_position + desk_width, y_position + desk_height};
            drawRectangle(topLeft, bottomRight);
        }
    }
}

void ClassroomEnvironment::drawClassroom(){
    drawWalls();
    drawExit();
    drawDesk();
    quit();
}