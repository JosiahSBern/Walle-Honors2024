#include "Environment.h"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <rclcpp/rclcpp.hpp> 
#include <iostream>
#include <cmath>  // Include for atan2

using namespace std;

// Constructor
Environment::Environment(rclcpp::Node::SharedPtr node) : node_(node) {
    // Initialize the pen client
    auto pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
}

void Environment::setExit(double x, double y, const string& direction) {
    exit.x = x;
    exit.y = y;
    this->direction = direction;
}


// Draw line between two points
void Environment::drawLine(Point start, Point end) {
    try {
        // Create teleport client
        static auto teleport_client = node_->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        // Move to start position with pen up
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        setPen(false);

        teleport_request->x = start.x;
        teleport_request->y = start.y;
        teleport_request->theta = atan2(end.y - start.y, end.x - start.x); // Calculate theta to point towards the end position

        // Send teleport request to move the turtle to the start position
        auto result = teleport_client->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Failed to teleport to start position");
        }

        // Set the pen down to start drawing
        setPen(true);

        // Calculate the total distance and determine how many steps to take
        double step_size = 0.1;
        double distance = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2)); // Euclidean distance
        int steps = distance / step_size;

        for (int i = 0; i < steps; i++) {
            teleport_request->x = start.x + (end.x - start.x) * i / steps;
            teleport_request->y = start.y + (end.y - start.y) * i / steps;

            result = teleport_client->async_send_request(teleport_request);
            if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
                throw std::runtime_error("Failed to draw line");
            }

            // Sleep for a short duration to slow down the drawing
            rclcpp::Rate rate(10); 
            rate.sleep();
        }

        // Move the turtle to the final endpoint
        teleport_request->x = end.x;
        teleport_request->y = end.y;
        result = teleport_client->async_send_request(teleport_request);
        if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("Failed to teleport to end position");
        }

        // Lift the pen
        setPen(false);
    } catch (const std::exception& e) {
        std::cout << "Error drawing line: " << e.what() << std::endl;
    }
}


RectangularRoom::RectangularRoom(rclcpp::Node::SharedPtr node) : Environment(node) {}

void RectangularRoom::drawWalls() {
    // Define room corners
    Point bottomLeft = {1.0, 1.0};
    Point bottomRight = {10.0, 1.0};
    Point topLeft = {1.0, 10.0};
    Point topRight = {10.0, 10.0};

    // Draw the walls
    drawLine(bottomLeft, bottomRight);  // Bottom wall
    drawLine(bottomRight, topRight);    // Right wall
    drawLine(topRight, topLeft);        // Top wall
    drawLine(topLeft, bottomLeft);      // Left wall
    
    RCLCPP_INFO(node_->get_logger(), "Room walls drawn successfully");
}

Point Environment::getExitPosition() {
    return exit;
}