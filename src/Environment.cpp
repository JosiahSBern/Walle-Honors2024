#include "Environment.h"
#include <cmath>

Environment::Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name) 
    : node_(node), turtle_name_(turtle_name) {
    std::string set_pen_service = "/" + turtle_name_ + "/set_pen";
    std::string teleport_service = "/" + turtle_name_ + "/teleport_absolute";

    pen_client_ = node_->create_client<turtlesim::srv::SetPen>(set_pen_service);
    teleport_client_ = node_->create_client<turtlesim::srv::TeleportAbsolute>(teleport_service);
}


void Environment::setExit(double x, double y, const std::string& direction) {
    exit.x = x;
    exit.y = y;
    this->direction = direction;
}

Point Environment::getExit() {
    return exit;
}

void Environment::drawLine(Point start, Point end) {
    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    setPen(false);

    teleport_request->x = start.x;
    teleport_request->y = start.y;
    teleport_request->theta = atan2(end.y - start.y, end.x - start.x);

    auto result = teleport_client_->async_send_request(teleport_request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to teleport to start position");
        return;
    }

    setPen(true);
    teleport_request->x = end.x;
    teleport_request->y = end.y;

    result = teleport_client_->async_send_request(teleport_request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to teleport to end position");
    }
    setPen(false);
}

void Environment::setPen(bool pen_state, int r, int g, int b, int width) {
    if (!pen_client_ || !pen_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Pen service not available for turtle: %s", turtle_name_.c_str());
        return;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = !pen_state;

    auto future = pen_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set pen for turtle: %s", turtle_name_.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "Pen set successfully for turtle: %s", turtle_name_.c_str());
    }
}


void Environment::drawRectangle(Point topLeft, Point bottomRight) {
    Point topRight = {bottomRight.x, topLeft.y};
    Point bottomLeft = {topLeft.x, bottomRight.y};
    drawLine(topLeft, topRight);
    drawLine(topRight, bottomRight);
    drawLine(bottomRight, bottomLeft);
    drawLine(bottomLeft, topLeft);
}

void Environment::quit() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down...");
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