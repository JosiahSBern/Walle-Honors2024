#include "TrashTurtle.h"
#include "Turtle.h"
#include <cmath>

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
                         double radius, TrashType type, const Point& target)
    : Turtle(node, name, radius), 
      type(type), 
      targetPosition(target), 
      targetRadius(0.5), 
      followingLeader_(false), 
      followDistanceThreshold_(2.0)  // Set a threshold distance for following
{
    // Initialization
}

void TrashTurtle::setLeaderTurtle(std::shared_ptr<Turtle> leader) {
    leaderTurtle = leader;
}

void TrashTurtle::followLeader() {
    if (!leaderTurtle) {
        RCLCPP_WARN(node_->get_logger(), "Leader not set for TrashTurtle: %s", name.c_str());
        return;
    }

    double distance_to_leader = calculateDistance(position, leaderTurtle->getPosition());
    if (distance_to_leader > followDistanceThreshold_) {
        RCLCPP_INFO(node_->get_logger(), "Leader is out of range. Stopping follow for: %s", name.c_str());
        followingLeader_ = false;
        stopMovement();
        return;
    }

    followingLeader_ = true;
    updateVelocityToTarget(leaderTurtle->getPosition());
}


void TrashTurtle::moveToBin() {
    if (followingLeader_) {
        followLeader();  // Follow the leader if within distance
    } else {
        // Stop when at the target bin
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        twist_pub_->publish(stop_msg);
    }
}

void TrashTurtle::move() {
    moveToBin();  // Reuse the moveToBin logic
}

void TrashTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "Rendering TrashTurtle: %s at (%f, %f)", 
                name.c_str(), position.x, position.y);
}

void TrashTurtle::updateVelocityToTarget(const Point& target) {
    double distance = calculateDistance(position, target);
    double angle = std::atan2(target.y - position.y, target.x - position.x);

    geometry_msgs::msg::Twist twist_msg;
    
    // Proportional control for linear velocity
    twist_msg.linear.x = std::min(distance, 1.0);  // Limit max speed
    
    // Proportional control for angular velocity to orient towards the target
    twist_msg.angular.z = angle * 0.5;  // Scaling factor to prevent over-rotation

    twist_pub_->publish(twist_msg);
}

bool TrashTurtle::isAtTarget() const {
    double distance = calculateDistance(position, targetPosition);
    return distance <= targetRadius;
}

void TrashTurtle::setTargetPosition(const Point& target) {
    targetPosition = target;
}

TrashType TrashTurtle::getTrashType() const {
    return type;
}

