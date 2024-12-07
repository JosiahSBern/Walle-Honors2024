#include "Turtle.h"

Turtle::Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius)
    : node_(node), name(name), radius(radius), position({0.0, 0.0}) 
{
    // Construct the cmd_vel topic name based on the turtle's name.
    // For a turtle named "teleop_turtle", this will be "teleop_turtle/cmd_vel".
    // Note: Turtlesim turtles typically have topics like "/turtleX/cmd_vel", 
    // but you do not usually need a leading slash.
    std::string cmd_vel_topic = name + "/cmd_vel";

    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(name + "/cmd_vel", 10);
    RCLCPP_INFO(node_->get_logger(), "Created Turtle '%s' with cmd_vel topic '%s'", name.c_str(), cmd_vel_topic.c_str());
}

Turtle::~Turtle() = default;

double Turtle::calculateDistance(const Point& p1, const Point& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool Turtle::checkCollision(const Point& other, double otherRadius) const {
    double distance = calculateDistance(position, other);
    // Adding a small safety margin (0.1) so turtles don't overlap visually.
    return distance <= (radius + otherRadius + 0.1); 
}

Point Turtle::getPosition() const {
    return position;
}

void Turtle::setPosition(const Point& newPosition) {
    position = newPosition;
}

std::string Turtle::getName() const {
   return name;
}

