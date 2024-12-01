#ifndef TURTLE_H
#define TURTLE_H

#include "Point.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Turtle {
protected:
    Point position;  // Current turtle position
    std::string name;  // Turtle's name
    double radius;  // Radius of the turtle's bounding circle

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    // Calculate the Euclidean distance between two points
    double calculateDistance(const Point& p1, const Point& p2) const {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

public:
    Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius)
        : node_(node), name(name), radius(radius), position({0.0, 0.0}) {
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    }

    virtual void move() = 0;
    virtual void renderTurtle() = 0;
    virtual ~Turtle() = default;

    bool checkCollision(const Point& other, double otherRadius) const {
        return calculateDistance(position, other) <= (radius + otherRadius);
    }

    Point getPosition() const { return position; }
    void setPosition(const Point& newPosition) { position = newPosition; }
};

#endif
