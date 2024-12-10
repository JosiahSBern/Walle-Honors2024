#include "Turtle.h"
#include "turtlesim/msg/pose.hpp"

Turtle::Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius) {
    this->node_ = node;
    this->name = name;
    this->radius = radius;

    this->twist_pub_ = this->node_->create_publisher<geometry_msgs::msg::Twist>("/" + name + "/cmd_vel", 10);

    // Initialize pose subscription to track updates for this turtle
    auto pose_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) {
        this->pose = *msg;  // Update the Pose data
        RCLCPP_INFO(this->node_->get_logger(), "Turtle %s Pose Updated: x=%.2f, y=%.2f, theta=%.2f", 
                    this->name.c_str(), pose.x, pose.y, pose.theta);
    };
    this->pose_sub_ = this->node_->create_subscription<turtlesim::msg::Pose>("/" + name + "/pose", 10, pose_callback);
}
Turtle::~Turtle() = default;

double Turtle::calculateDistance(const Point& p1, const Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool Turtle::checkCollision(const Point& other, double otherRadius) const {
    double distance = calculateDistance(position, other);
    return distance <= (radius + otherRadius + 0.1); 
}

Point Turtle::getPosition() const {
    return {pose.x, pose.y};
}

void Turtle::setPosition(const Point& newPosition) {
    pose.x = newPosition.x;
    pose.y = newPosition.y;
}

double Turtle::getOrientation() const {
    return pose.theta;
}

void Turtle::setOrientation(double theta) {
    pose.theta = theta;
}

turtlesim::msg::Pose Turtle::getPose() const {
    return pose;
}