#include "Environment.h"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <iostream>

using namespace std;

//Constructor
Environment::Environment(rclcpp::Node::SharedPtr node) : node_(node) {
    // Initialize the pen client
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
}

void Environment::setExit(double x,double y, string& direction){
    exit.x = x;
    exit.y = y;
    this->direction = direction;
}

//Draw line between two points
void Environment::drawLine(Point start, Point end) {
    // Create teleport client
    auto teleport_client = node_->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    
    // Move to start position with pen up
    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    setPen(false);
    
    teleport_request->x = start.x;
    teleport_request->y = start.y;
    teleport_request->theta = atan2(end.y - start.y, end.x - start.x);
    
    auto result = teleport_client->async_send_request(teleport_request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Failed to teleport to start position");
    }

    // Put pen down and move to end position
    setPen(true);
    teleport_request->x = end.x;
    teleport_request->y = end.y;
    
    result = teleport_client->async_send_request(teleport_request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Failed to draw line");
    }
    
    setPen(false);
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

void Environment::setPen(bool on) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    
    if (on) {
        request->r = 0;    // Black color
        request->g = 0;
        request->b = 0;
        request->width = 2;
        request->off = 0;  // Pen on
    } else {
        request->off = 1;  // Pen off
    }

    auto result = pen_client_->async_send_request(request);
    
    // Wait for the result
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Failed to set pen state");
    }
}