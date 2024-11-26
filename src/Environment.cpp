#include "Environment.h"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <iostream>
#include <cmath>  
using namespace std;

// Constructor
Environment::Environment(rclcpp::Node::SharedPtr node) : node_(node) {
    //Initialize the pen client
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    // Initialize the teleport client
    teleport_client_ = node_->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
}

//Set exit point
void Environment::setExit(double x, double y, const string& direction) {
    exit.x = x;
    exit.y = y;
    this->direction = direction;
}

Point Environment::getExit() {
    return exit;
}

//Draw line between points
void Environment::drawLine(Point start, Point end) {
    try {
        //Create a teleport request to move the turtle to the start position
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        setPen(false);  //Lift Pen

        //Set start position and angle based on the direction to the end point
        teleport_request->x = start.x;
        teleport_request->y = start.y;
        teleport_request->theta = atan2(end.y - start.y, end.x - start.x);

        //Move the turtle to the start position
        auto result = teleport_client_->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw runtime_error("Failed to teleport to start position");
        }

        //Set the pen down to start drawing
        setPen(true);

        //Move to the final endpoint
        teleport_request->x = end.x;
        teleport_request->y = end.y;
        result = teleport_client_->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw runtime_error("Failed to teleport to end position");
        }

        //Lift the pen after drawing the line
        setPen(false);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error drawing line: %s", e.what());
    }
}


// Function to set pen on or off
void Environment::setPen(bool pen_state, int r, int g, int b, int width) {
    //Create the pen client
    auto pen_client = node_->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

    // Wait for the service to be available
    while (!pen_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }

    // Prepare the request
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = !pen_state;

    // Send the request and get the result future
    auto future = pen_client->async_send_request(request);

    // Wait for the result
    auto result = rclcpp::spin_until_future_complete(node_, future);

    // Check the result
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Pen settings updated successfully");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set pen");
    }
}
//Draw Rectangles with TurtleSim
void Environment::drawRectangle(Point topLeft, Point bottomRight) {
    Point topRight = {bottomRight.x, topLeft.y};
    Point bottomLeft = {topLeft.x, bottomRight.y};
    drawLine(topLeft, topRight);   //Top wall
    drawLine(topRight, bottomRight);  //Right wall
    drawLine(bottomRight, bottomLeft);  //Bottom wall
    drawLine(bottomLeft, topLeft);  //Left wall
}
//Quit Program
void Environment::quit() {
    std::cout << "Shutting down..." << std::endl;
    rclcpp::shutdown(); 
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