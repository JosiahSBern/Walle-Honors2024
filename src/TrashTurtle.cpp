#include "TrashTurtle.h"
#include "turtlesim/srv/set_pen.hpp"
#include "Turtle.h"
#include <cmath>

// Constructor
TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
                         double radius, TrashType type, const Point& target)
    : Turtle(node, name, radius), 
      type(type), 
      targetPosition(target), 
      targetRadius(0.5), 
      followingLeader_(false), 
      followDistanceThreshold_(2.0),
      leaderTurtle(nullptr) {  // Ensure leaderTurtle is initialized
    // Initialize the pen client for this turtle
    pen_client_ = node->create_client<turtlesim::srv::SetPen>("/" + name + "/set_pen");
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s initialized.", name.c_str());
}

// Set the leader turtle
void TrashTurtle::setLeaderTurtle(std::shared_ptr<Turtle> leader) {
    leaderTurtle = leader;
    followingLeader_ = true;  // Start following
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is now following %s.", name.c_str(), leader->getName().c_str());
}

// Follow the leader
void TrashTurtle::followLeader() {
    if (!leaderTurtle) {
        RCLCPP_WARN(node_->get_logger(), "Leader not set for TrashTurtle: %s", name.c_str());
        followingLeader_ = false;
        return;
    }

    double distance_to_leader = calculateDistance(position, leaderTurtle->getPosition());
    if (distance_to_leader > followDistanceThreshold_) {
        RCLCPP_INFO(node_->get_logger(), "Leader out of range. Stopping follow for TrashTurtle: %s", name.c_str());
        followingLeader_ = false;
        stopMovement();
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is following leader.", name.c_str());
    updateVelocityToTarget(leaderTurtle->getPosition());
}

// Move to assigned bin or follow leader
void TrashTurtle::moveToBin() {
    if (followingLeader_) {
        followLeader();
    } else if (!isAtTarget()) {
        RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s moving to bin.", name.c_str());
        updateVelocityToTarget(targetPosition);
    } else {
        stopMovement();
        RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s reached its bin.", name.c_str());
    }
}

// Reuse moveToBin for move
void TrashTurtle::move() {
    moveToBin();
}

// Update velocity to target
void TrashTurtle::updateVelocityToTarget(const Point& target) {
    double distance = calculateDistance(position, target);
    double angle = std::atan2(target.y - position.y, target.x - position.x);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = std::min(distance, 1.0);  // Limit max speed
    twist_msg.angular.z = angle * 0.5;  // Scale angular speed
    twist_pub_->publish(twist_msg);

    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s moving towards (%f, %f).", name.c_str(), target.x, target.y);
}

// Render turtle information
void TrashTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "Rendering TrashTurtle %s at (%f, %f).", 
                name.c_str(), position.x, position.y);
    if (followingLeader_) {
        RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is following a leader.", name.c_str());
    } else if (isAtTarget()) {
        RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is at its bin.", name.c_str());
    }
}

// Stop movement
void TrashTurtle::stopMovement() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    twist_pub_->publish(stop_msg);
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s has stopped moving.", name.c_str());
}

// Check if turtle is at target
bool TrashTurtle::isAtTarget() const {
    double distance = calculateDistance(position, targetPosition);
    return distance <= targetRadius;
}

// Set target position
void TrashTurtle::setTargetPosition(const Point& target) {
    targetPosition = target;
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s new target set: (%f, %f).", name.c_str(), target.x, target.y);
}

// Set pen color
void TrashTurtle::setPenColor(int r, int g, int b, int width) {
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
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set pen color for TrashTurtle: %s", name.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "Pen color set for TrashTurtle: %s to (%d, %d, %d, %d).",
                    name.c_str(), r, g, b, width);
    }
}
