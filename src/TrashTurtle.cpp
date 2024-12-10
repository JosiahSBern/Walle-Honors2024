// src/TrashTurtle.cpp
#include "TrashTurtle.h"
#include <cmath>
#include <limits>
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type, const Point& target)
    : Turtle(node, name, radius) // Base class constructor must remain in initializer list
{
    this->type = type;
    this->targetPosition = target;
    this->targetRadius = 0.5;
    this->currentState_ = SortState::MOVING_TO_BIN;
    this->pen_client_ = this->node_->create_client<turtlesim::srv::SetPen>("/" + name + "/set_pen");
    this->teleport_client_ = this->node_->create_client<turtlesim::srv::TeleportAbsolute>("/" + name + "/teleport_absolute");
    this->twist_pub_ = this->node_->create_publisher<geometry_msgs::msg::Twist>("/" + name + "/cmd_vel", 10);

    // Subscribe to the turtle's pose topic to track its position and orientation
    auto pose_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) {
        this->position.x = msg->x;
        this->position.y = msg->y;
        this->orientation = msg->theta;  // Update orientation
    };

    this->node_->create_subscription<turtlesim::msg::Pose>("/" + name + "/pose", 10, pose_callback);
}
 


bool TrashTurtle::isAtTarget() const {
    double distance = calculateDistance(position, targetPosition);
    return distance <= targetRadius;  // Check if within radius
}
void TrashTurtle::move(const Turtle& target, double follow_distance) {
    // Get the target position and orientation
    Point targetPosition = target.getPosition();
    double targetOrientation = target.getOrientation();

    // Calculate the desired position to maintain the follow distance
    double desired_x = targetPosition.x - follow_distance * std::cos(targetOrientation);
    double desired_y = targetPosition.y - follow_distance * std::sin(targetOrientation);

    // Calculate the distance and angle to the desired position
    double dx = desired_x - position.x;
    double dy = desired_y - position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_angle = std::atan2(dy, dx);

    // Log for debugging
    RCLCPP_INFO(node_->get_logger(), "%s: Desired Position=(%.2f, %.2f), Distance=%.2f, Target Angle=%.2f",
                name.c_str(), desired_x, desired_y, distance, target_angle);

    // Use TeleportAbsolute to move to the desired position
    if (teleport_client_->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = desired_x;
        request->y = desired_y;
        request->theta = target_angle;

        auto result = teleport_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            position.x = desired_x;
            position.y = desired_y;
            orientation = target_angle;
            RCLCPP_INFO(node_->get_logger(), "%s moved to (%.2f, %.2f) with orientation %.2f.",
                        name.c_str(), request->x, request->y, request->theta);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to move %s to the desired position.", name.c_str());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "TeleportAbsolute service not available for %s.", name.c_str());
    }
}











void TrashTurtle::sortIntoBin() {
    if (currentState_ == SortState::MOVING_TO_BIN) {
        currentState_ = SortState::SORTED;
        stopMovement();  // Stop the turtle's motion
        RCLCPP_INFO(node_->get_logger(), "%s has been sorted!", name.c_str());
        setPenColor(0, 255, 0, 2);  // Green pen for sorted turtles

        // Optional: Display a success message for the player
        displaySuccessMessage();
    }
}

void TrashTurtle::displaySuccessMessage() {
    RCLCPP_INFO(node_->get_logger(), "%s is successfully sorted into the correct bin!", name.c_str());
}


void TrashTurtle::updateVelocityToTarget(const Point& target) {
    double distance = calculateDistance(position, target);
    double angle = std::atan2(target.y - position.y, target.x - position.x);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = std::min(distance * 0.5, 1.0);  // Proportional speed control
    twist_msg.angular.z = angle * 0.5;                // Proportional rotation control

    twist_pub_->publish(twist_msg);
}

Point TrashTurtle::getBinPositionForTrashType() const {
    // Define bin positions similar to your GameEnvironment
    switch(type) {
        case TrashType::TRASH:
            return {1.5, 9.0};  // First bin (green)
        case TrashType::RECYCLING:
            return {4.5, 9.0};  // Second bin (blue)
        case TrashType::PAPER:
            return {7.5, 9.0};  // Third bin (gray)
        default:
            // Default fallback
            return {5.5, 5.5};
    }
}

void TrashTurtle::setTargetPosition(const Point& target) {
    targetPosition = target;
}

void TrashTurtle::setPenColor(int r, int g, int b, int width) {
    auto set_pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
    set_pen_request->r = r;
    set_pen_request->g = g;
    set_pen_request->b = b;
    set_pen_request->width = width;
    set_pen_request->off = 0;

    auto result = pen_client_->async_send_request(set_pen_request);
    // Optionally, handle the response or future
}

TrashType TrashTurtle::getTrashType() const {
    return type;
}

void TrashTurtle::stopMovement() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    twist_pub_->publish(stop_msg);
}


SortState TrashTurtle::getCurrentState() const {
    return currentState_;
}

