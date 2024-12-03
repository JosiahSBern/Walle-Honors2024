#include "TrashTurtle.h"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type, const Point& target, double targetRadius)
    : Turtle(node, name, radius), type(type), targetPosition(target), targetRadius(targetRadius) {}

void TrashTurtle::updateVelocityToTarget(const Point& target) {
    double distance = Turtle::calculateDistance(position, target);

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = distance > 0.1 ? 1.0 : 0.0;  // Move towards the target
    twist_msg.angular.z = 0.0;

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
    targetPosition = target;
}

TrashType TrashTurtle::getTrashType() const {
    return type;
}
