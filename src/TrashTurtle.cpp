#include "TrashTurtle.h"
#include "turtlesim/srv/set_pen.hpp"
#include "Turtle.h"
#include <cmath>

TrashTurtle::TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
                         double radius, TrashType type, const Point& target)
    : Turtle(node, name, radius), 
      type(type), 
      targetPosition(target), 
      targetRadius(0.5), 
      followingLeader_(false), 
      followDistanceThreshold_(2.0),
      leaderTurtle(nullptr) {
    
    // Validate inputs
    if (!node) {
        throw std::invalid_argument("Node cannot be null");
    }

    pen_client_ = node->create_client<turtlesim::srv::SetPen>("/" + name + "/set_pen");
    
    if (!pen_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create pen client for TrashTurtle %s", name.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s initialized successfully.", name.c_str());
    }
}

void TrashTurtle::setLeaderTurtle(std::shared_ptr<Turtle> leader) {
    if (!leader) {
        RCLCPP_WARN(node_->get_logger(), "Attempted to set null leader for TrashTurtle: %s", name.c_str());
        return;
    }

    leaderTurtle = leader;
    followingLeader_ = true;
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is now following %s.", name.c_str(), leader->getName().c_str());
}

void TrashTurtle::move() {
    if (followingLeader_) {
        followLeader();
    } else {
        moveToBin();
    }
}

void TrashTurtle::followLeader() {
    if (!leaderTurtle) {
        RCLCPP_WARN(node_->get_logger(), "No leader set for TrashTurtle: %s", name.c_str());
        followingLeader_ = false;
        return;
    }

    Point leaderPos = leaderTurtle->getPosition();
    double distance_to_leader = calculateDistance(position, leaderPos);

    if (distance_to_leader > followDistanceThreshold_) {
        RCLCPP_DEBUG(node_->get_logger(), "Leader out of range for TrashTurtle: %s", name.c_str());
        followingLeader_ = false;
        stopMovement();
        return;
    }

    updateVelocityToTarget(leaderPos);
}

void TrashTurtle::moveToBin() {
    if (isAtTarget()) {
        RCLCPP_DEBUG(node_->get_logger(), "TrashTurtle %s reached its bin.", name.c_str());
        stopMovement();
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "TrashTurtle %s moving to bin.", name.c_str());
    updateVelocityToTarget(targetPosition);
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
    twist_msg.linear.x = std::min(distance, 1.0);  // Limit max speed
    twist_msg.angular.z = angle * 0.5;  // Scale angular speed

    twist_pub_->publish(twist_msg);

    RCLCPP_DEBUG(node_->get_logger(), "TrashTurtle %s moving towards (%f, %f).", 
                 name.c_str(), target.x, target.y);
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

    // Additional color validation
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        RCLCPP_WARN(node_->get_logger(), "Invalid color values for TrashTurtle: %s", name.c_str());
        return;
    }

    auto set_pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
    set_pen_request->r = r;
    set_pen_request->g = g;
    set_pen_request->b = b;
    set_pen_request->width = width;
    set_pen_request->off = 0;  // Pen ON

    auto result = pen_client_->async_send_request(
        set_pen_request,
        [this, r, g, b, width](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(node_->get_logger(), "Pen color set for TrashTurtle: %s to (%d, %d, %d, %d).",
                            name.c_str(), r, g, b, width);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set pen color: %s", e.what());
            }
        }
    );
}
void TrashTurtle::stopMovement() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    twist_pub_->publish(stop_msg);
    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s has stopped moving.", name.c_str());
}

bool TrashTurtle::isAtTarget() const {
    double distance = calculateDistance(position, targetPosition);
    return distance <= targetRadius;
}

void TrashTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "Rendering TrashTurtle: %s at (%f, %f)",
                name.c_str(), position.x, position.y);
}
