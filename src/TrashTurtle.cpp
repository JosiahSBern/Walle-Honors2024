#include "TrashTurtle.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, 
                         TrashType type, const Point& boxPosition, double speed)
    : Turtle(node, name, radius), trashType(type), boxPosition(boxPosition), speed(speed) {}

// Update the velocity to move towards the target
void TrashTurtle::updateVelocityToTarget(const Point& target) {
    geometry_msgs::msg::Twist twist_msg;
    double dx = target.x - position.x;
    double dy = target.y - position.y;
    double distance = calculateDistance(position, target);

    if (distance > 0.1) {  // Threshold to prevent oscillations
        twist_msg.linear.x = speed * (dx / distance);
        twist_msg.linear.y = speed * (dy / distance);
    } else {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
    }
    twist_pub_->publish(twist_msg);
}

// Movement logic for the turtle
void TrashTurtle::move() {
    if (calculateDistance(position, targetPosition) > 0.1) {
        updateVelocityToTarget(targetPosition);
    } else {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_pub_->publish(twist_msg);
    }
}

// Render turtle details for visualization
void TrashTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "%s at (%.2f, %.2f) - TrashType: %d", 
                name.c_str(), position.x, position.y, static_cast<int>(trashType));
}

// Move to the designated bin
void TrashTurtle::moveToBin() {
    setTargetPosition(boxPosition);
}

// Set the turtle's target position
void TrashTurtle::setTargetPosition(const Point& target) {
    targetPosition = target;
}

// Get the trash type
TrashType TrashTurtle::getTrashType() const {
    return trashType;
}
