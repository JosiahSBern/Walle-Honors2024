#include "Environment.h"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>  
using namespace std;

// Constructor
Environment::Environment(rclcpp::Node::SharedPtr node) : node_(node) {
    // Initialize the pen client
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    // Initialize the teleport client
    teleport_client_ = node_->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

    // Initialize the spawn client for new turtle
    // spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");
}

void Environment::setExit(double x, double y, const string& direction) {
    exit.x = x;
    exit.y = y;
    this->direction = direction;
}
Point Environment::getExit() {
    return exit;
}
//Draw line between two points
void Environment::drawLine(Point start, Point end) {
    try {
        //Move to start position with pen up
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        setPen(false);

        teleport_request->x = start.x;
        teleport_request->y = start.y;
        teleport_request->theta = atan2(end.y - start.y, end.x - start.x);

        //Send teleport request to move the turtle to the start position
        auto result = teleport_client_->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Failed to teleport to start position");
        }

        //Set the pen down to start drawing
        setPen(true);

        //Calculate the total distance and determine how many steps to take
        double step_size = 5;
        double distance = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
        int steps = distance / step_size;

        for (int i = 0; i < steps; i++) {
            teleport_request->x = start.x + (end.x - start.x) * i / steps;
            teleport_request->y = start.y + (end.y - start.y) * i / steps;

            result = teleport_client_->async_send_request(teleport_request);
            if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
                throw std::runtime_error("Failed to draw line");
            }

            //Sleep for a short duration to slow down the drawing
            rclcpp::Rate rate(10);
            rate.sleep();
        }

        //Move the turtle to the final endpoint
        teleport_request->x = end.x;
        teleport_request->y = end.y;
        result = teleport_client_->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Failed to teleport to end position");
        }

        // Lift the pen
        setPen(false);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error drawing line: %s", e.what());
    }
}

// Function to set pen on or off
void Environment::setPen(bool on) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 255;
    request->g = 255;
    request->b = 255;
    request->width = 2;
    request->off = !on;

    auto result = pen_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Failed to set pen");
    }
}

void Environment::setColor(int r, int g, int b) {
    // auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    // request->r = r; 
    // request->g = g; 
    // request->b = b;
    // request->width = 2;  
    // request->off = false;

    auto result = pen_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Failed to set pen color");
    }
}

void Environment::drawRectangle(Point topLeft, Point bottomRight) {
    Point topRight = {bottomRight.x, topLeft.y};
    Point bottomLeft = {topLeft.x, bottomRight.y};
    drawLine(topLeft, topRight);   //Top wall
    drawLine(topRight, bottomRight);  //Right wall
    drawLine(bottomRight, bottomLeft);  //Bottom wall
    drawLine(bottomLeft, topLeft);  //Left wall
}

void Environment::quit() {
    std::cout << "Shutting down..." << std::endl;
    rclcpp::shutdown();  // Shutdown ROS2
}
// RectangularRoom::RectangularRoom(rclcpp::Node::SharedPtr node) : Environment(node) {}

// void RectangularRoom::drawWalls() {
//     // Define room corners
//     Point bottomLeft = {1.0, 1.0};
//     Point bottomRight = {10.0, 1.0};
//     Point topLeft = {1.0, 10.0};
//     Point topRight = {10.0, 10.0};

//     // Draw the walls
//     drawLine(bottomLeft, bottomRight);  // Bottom wall
//     drawLine(bottomRight, topRight);    // Right wall
//     drawLine(topRight, topLeft);        // Top wall
//     drawLine(topLeft, bottomLeft);      // Left wall
    
//     RCLCPP_INFO(node_->get_logger(), "Room walls drawn successfully");
//     quit();
// }

ClassroomEnvironment::ClassroomEnvironment(rclcpp::Node::SharedPtr node)
    : Environment(node) {
     roomLength = 11.0;
     roomWidth = 8.0;
     desk_width = (roomWidth - 4) / 5;;
     desk_height= 1.0;
     desk_spacing = 0.5;

} 
    void ClassroomEnvironment::drawWalls() {
    //Define the corners of the classroom
    Point bottomLeft = {1.0, 1.0};
    Point bottomRight = {roomWidth, 1.0};
    Point topLeft = {1.0, roomLength};
    Point topRight = {roomWidth, roomLength};

    // Draw the classroom walls
    drawLine(bottomLeft, bottomRight);//Bottom wall
    drawLine(bottomRight, topRight);//Right wall
    drawLine(topRight, topLeft);// Top wall
    drawLine(topLeft, bottomLeft);// Left wall
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
    setColor(0, 255, 0);
      
    Point exitPosition = {roomWidth, roomLength};
    drawLine(exitPosition, {exitPosition.x + 0.5, exitPosition.y});
    
    // Set the exit coordinates
    setExit(exitPosition.x + 0.5, exitPosition.y, "north");
    RCLCPP_INFO(node_->get_logger(), "Exit position set at: (%f, %f)", exitPosition.x + 0.5, exitPosition.y);
}

void ClassroomEnvironment::drawDesk() {
    // Make sure to wait for the pen service to be available
    while (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for pen service");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for pen service...");
    }

    // Set the pen color to brown
    setColor(139, 69, 19);
    
    // int desksPerRow = (roomWidth - 2) / (desk_width + desk_spacing);
    // int desksPerColumn = (roomLength - 2) / (desk_height + desk_spacing);
    for (int row = 0; row < 3; ++row) {
    double y_position;
    
    // Determine y_position based on row index:
    if (row == 0) { 
        y_position = 1.0;  // Row 1 touches the left wall
    } else if (row == 2) { 
        y_position = roomLength - desk_height - 1.0;  // Row 3 near the right wall
    } else {
        y_position = roomLength / 2 - (desk_height / 2);  // Center row
    }
    
    // Loop through each column in the row
    for (int col = 0; col < desksPerRow; ++col) {
        double x_position = 1.0 + col * (desk_width + desk_spacing);
        
        // Draw the desk as a rectangle from (x_position, y_position)
        Point topLeft = {x_position, y_position};
        Point bottomRight = {topLeft.x + desk_width, topLeft.y + desk_height};
        drawRectangle(topLeft, bottomRight);
    }
}
}



// void ClassroomEnvironment::spawnStartingTurtle() {
//     auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    
//     // Position the new turtle at bottom right
//     request->x = roomWidth - 1.0;  // Slightly off from the wall
//     request->y = 1.5;              // Slightly above the bottom wall
//     request->theta = M_PI / 2;     // Facing up (90 degrees)
//     request->name = "turtle2";

//     auto result = spawn_client_->async_send_request(request);
    
//     if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_INFO(node_->get_logger(), "Successfully spawned turtle2 at starting position");
//     } else {
//         RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle2");
//     }
// }

void ClassroomEnvironment::drawClassroom(){
    drawWalls();
    drawExit();
    drawDesk();
    // spawnStartingTurtle();
    quit();
}
