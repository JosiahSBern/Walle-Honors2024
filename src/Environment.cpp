#include "Environment.h"
#include <cmath>

Environment::Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name)
    : turtle_name(turtle_name), node_(node) {
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>("/" + turtle_name + "/set_pen");
    teleport_client_ = node_->create_client<turtlesim::srv::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");
    spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");

    // Wait for services during initialization
    if (!pen_client_->wait_for_service(std::chrono::seconds(5)) ||
        !teleport_client_->wait_for_service(std::chrono::seconds(5)) ||
        !spawn_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "One or more services are not available. Check turtlesim node.");
    }
}

void Environment::drawLine(Point start, Point end, bool pen_state, int r, int g, int b, int width) {
    setPen(false, 0, 0, 0, 1); // Disable pen 

    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    teleport_request->x = start.x;
    teleport_request->y = start.y;
    teleport_request->theta = atan2(end.y - start.y, end.x - start.x);

    if (rclcpp::spin_until_future_complete(node_, teleport_client_->async_send_request(teleport_request)) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to teleport to start position");
        return;
    }

    setPen(pen_state, r, g, b, width); // Enable pen for drawing
    teleport_request->x = end.x;
    teleport_request->y = end.y;

    if (rclcpp::spin_until_future_complete(node_, teleport_client_->async_send_request(teleport_request)) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to teleport to end position");
    }

    setPen(false, 0, 0, 0, 1); // Disable pen after drawing
}

void Environment::setPen(bool pen_state, int r, int g, int b, int width) {
    if (!pen_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Pen service not available.");
        return;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;                     // Red value (uint8)
    request->g = g;                     // Green value (uint8)
    request->b = b;                     // Blue value (uint8)
    request->width = width;             // Pen width (uint8)
    request->off = static_cast<uint8_t>(!pen_state);  // Convert bool to uint8 (0 for enabled, 1 for disabled)

    auto result = pen_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set pen.");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Pen state set: r=%d, g=%d, b=%d, width=%d, off=%d",
                    request->r, request->g, request->b, request->width, request->off);
    }
}

void Environment::drawRectangle(Point topLeft, Point bottomRight, int r, int g, int b) {
    drawLine(topLeft, {bottomRight.x, topLeft.y}, true, r, g, b, 2);
    drawLine({bottomRight.x, topLeft.y}, bottomRight, true, r, g, b, 2);
    drawLine(bottomRight, {topLeft.x, bottomRight.y}, true, r, g, b, 2);
    drawLine({topLeft.x, bottomRight.y}, topLeft, true, r, g, b, 2);
}

void Environment::spawnTurtle(const std::string& name, double x, double y, double theta) {
    if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Spawn service not available.");
        return;
    }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = name;
    request->x = x;
    request->y = y;
    request->theta = theta;

    auto future = spawn_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle %s", name.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "Turtle %s spawned at (%f, %f)", name.c_str(), x, y);
    }
}
