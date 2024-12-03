#include "TrashTurtle.h"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type, const Point& target, double targetRadius)
    : Turtle(node, name, radius), type(type), targetPosition(target), targetRadius(targetRadius) {}

void TrashTurtle::updateVelocityToTarget(const Point& target) {
    // Consider adding more sophisticated movement
    double distance = Turtle::calculateDistance(position, target);
    double angle = std::atan2(target.y - position.y, target.x - position.x);

    geometry_msgs::msg::Twist twist_msg;
    // Proportional control for linear velocity
    twist_msg.linear.x = std::min(distance, 1.0);  // Limit max speed
    // Add angular velocity to orient towards the target
    twist_msg.angular.z = angle;

    twist_pub_->publish(twist_msg);
}
void TrashTurtle::move() {
    if (Turtle::calculateDistance(position, targetPosition) > 0.1) {
        updateVelocityToTarget(targetPosition);
    } else {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        twist_pub_->publish(stop_msg);
    }
}

void TrashTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "Rendering TrashTurtle: %s at (%f, %f)", name.c_str(), position.x, position.y);
}

void TrashTurtle::setTargetPosition(const Point& target) {
    if (std::isnan(target.x) || std::isnan(target.y)) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid target position");
        return;
    }
    targetPosition = target;
}

TrashType TrashTurtle::getTrashType() const {
    return type;
}
