#include "Turtle.h"

// Constructor
Turtle::Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius)
    : node_(node), name(name), radius(radius), position({0.0, 0.0}) {
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
}

// Destructor
Turtle::~Turtle() = default;

// Calculate Euclidean distance between two points
double Turtle::calculateDistance(const Point& p1, const Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Check for collision between turtles
bool Turtle::checkCollision(const Point& other, double otherRadius) const {
    return calculateDistance(position, other) <= (radius + otherRadius);
}

// Get the current position of the turtle
Point Turtle::getPosition() const {
    return position;
}

// Set a new position for the turtle
void Turtle::setPosition(const Point& newPosition) {
    position = newPosition;
}
