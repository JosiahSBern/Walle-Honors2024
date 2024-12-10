// src/TrashTurtle.cpp
#include "TrashTurtle.h"
#include <cmath>
#include <limits>
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <random>  // For randomization of trash type
#include "Point.h"

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type, const Point& target)
    : Turtle(node, name, radius) // Base class constructor must remain in initializer list
{
    this->targetPosition = target;
    this->targetRadius = 0.5;
    this->currentState_ = SortState::MOVING_TO_BIN;
    this->pen_client_ = this->node_->create_client<turtlesim::srv::SetPen>("/" + name + "/set_pen");
    this->teleport_client_ = this->node_->create_client<turtlesim::srv::TeleportAbsolute>("/" + name + "/teleport_absolute");
    this->twist_pub_ = this->node_->create_publisher<geometry_msgs::msg::Twist>("/" + name + "/cmd_vel", 10);


    if (type == TrashType::NONE) {
        // Randomly assign a trash type
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 2);  // Randomize TrashType: 0, 1, or 2
        this->type = static_cast<TrashType>(dis(gen));  // Randomize the trash type
    } else {
        this->type = type;
    }

    // Set the pen color based on the trash type
    setPenColorForTrashType();

    // Subscribe to the turtle's pose topic to track its position and orientation
    auto pose_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) {
        this->position.x = msg->x;
        this->position.y = msg->y;
        this->orientation = msg->theta;  // Update orientation
    };

    this->node_->create_subscription<turtlesim::msg::Pose>("/" + name + "/pose", 10, pose_callback);
}

void TrashTurtle::setPenColorForTrashType() {
    int r, g, b;
    switch(type) {
        case TrashType::TRASH:
            r = 0; g = 100; b = 0;  // Red for Trash
            break;
        case TrashType::RECYCLING:
            r = 0; g = 0; b = 255;  // Green for Recycling
            break;
        case TrashType::PAPER:
            r = 128; g = 128; b = 128;  // Blue for Paper
            break;
        default:
            r = 255; g = 255;b = 255;  // Default to white
            break;
    }
    setPenColor(r, g, b, 2);  // Set the color and pen width
}

void TrashTurtle::setPenColor(int r, int g, int b, int width) {
    if (!pen_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Pen client not initialized for %s", name.c_str());
        return;
    }

    if (!pen_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "SetPen service not available for %s.", name.c_str());
        return;
    }

    auto set_pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
    set_pen_request->r = r;
    set_pen_request->g = g;
    set_pen_request->b = b;
    set_pen_request->width = width;
    set_pen_request->off = 0;  // Pen ON

    auto result = pen_client_->async_send_request(set_pen_request);
}

Point TrashTurtle::getBinPositionForTrashType() const {
    // Define bin positions similar to your GameEnvironment
    switch(type) {
        case TrashType::TRASH:
            return {1.5, 9.0};  // First bin 
        case TrashType::RECYCLING:
            return {4.5, 9.0};  // Second bin 
        case TrashType::PAPER:
            return {7.5, 9.0};  // Third bin 
        default:
            return {5.5, 5.5};  
    }
}


void TrashTurtle::move(const Turtle& target, double follow_distance) {
    // Get the target position and orientation
    if (getCurrentState() == SortState::MOVING_TO_BIN){
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
}






void TrashTurtle::updateVelocityToTarget(const Point& target) {
    // Validate target
    if (std::isnan(target.x) || std::isnan(target.y)) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid target position for TrashTurtle: %s", name.c_str());
        return;
    }

    double distance = calculateDistance(position, target);
    double angle = std::atan2(target.y - position.y, target.x - position.x);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = std::min(distance * 0.5, 1.0);  // Proportional speed control
    twist_msg.angular.z = angle * 0.5;                // Proportional rotation control

    twist_pub_->publish(twist_msg);

    RCLCPP_DEBUG(node_->get_logger(), "TrashTurtle %s moving towards (%f, %f).", 
                 name.c_str(), target.x, target.y);
}




void TrashTurtle::setTargetPosition(const Point& target) {
    targetPosition = target;
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

void TrashTurtle::setCurrentState(SortState newState) {
    currentState_ = newState;  
    switch (currentState_) {
        case SortState::MOVING_TO_BIN:
            RCLCPP_INFO(node_->get_logger(), "%s is moving to bin.", name.c_str());
            break;
        case SortState::SORTED:
            RCLCPP_INFO(node_->get_logger(), "%s has been sorted.", name.c_str());
            break;
    }
}

void TrashTurtle::teleportToBinCenter() {
    if (teleport_client_->wait_for_service(std::chrono::seconds(1))) {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = targetPosition.x;
        request->y = targetPosition.y;
        request->theta = 0.0;  // Assuming no rotation needed at the center

        auto result = teleport_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "%s teleported to the bin center at (%.2f, %.2f).", name.c_str(), targetPosition.x, targetPosition.y);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to teleport %s to the bin center.", name.c_str());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Teleport service not available for %s.", name.c_str());
    }
}

bool TrashTurtle::isAtTarget() const {
    double distance = calculateDistance(position, targetPosition);
    return distance <= targetRadius;  // Check if within radius
}

// Function to calculate distance
double TrashTurtle::calculateDistance(const Point& p1, const Point& p2) const {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

void TrashTurtle::stopAtTarget() {
    // Check if we are within the target radius (center of the bin)

    double distance = calculateDistance(position, targetPosition);
    if (distance <= targetRadius) {
        // Teleport the TrashTurtle to the exact position of the bin center
        setCurrentState(SortState::SORTED); 
        teleportToBinCenter();
        stopMovement();  // Stop movement after reaching the bin
        RCLCPP_INFO(node_->get_logger(), "%s has stopped at the target bin.", name.c_str());
    }
}