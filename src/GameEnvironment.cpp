#include "Environment.h"
#include <cmath>

Environment::Environment(rclcpp::Node::SharedPtr node) : node_(node) {
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    teleport_client_ = node_->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

    if (!pen_client_ || !teleport_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize service clients.");
    }
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
    RCLCPP_INFO(node_->get_logger(), "Drawing line from (%f, %f) to (%f, %f)", start.x, start.y, end.x, end.y);
    if (!teleport_client_ || !teleport_client_->service_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "Teleport service is unavailable.");
        return;
    }

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
    if (!pen_client_ || !pen_client_->service_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "Pen service not available.");
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
        RCLCPP_ERROR(node_->get_logger(), "Failed to set pen");
    }
}

void Environment::drawRectangle(Point topLeft, Point bottomRight) {
    RCLCPP_INFO(node_->get_logger(), "Drawing rectangle: TopLeft (%f, %f), BottomRight (%f, %f)", 
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    Point topRight = {bottomRight.x, topLeft.y};
    Point bottomLeft = {topLeft.x, bottomRight.y};
    drawLine(topLeft, topRight);
    drawLine(topRight, bottomRight);
    drawLine(bottomRight, bottomLeft);
    drawLine(bottomLeft, topLeft);
}

void Environment::quit() {
    if (rclcpp::ok()) {
        RCLCPP_INFO(node_->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Node is already shut down.");
    }
}
