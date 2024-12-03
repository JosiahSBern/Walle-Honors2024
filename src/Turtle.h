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

public:
    Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius)
        : node_(node), name(name), radius(radius), position({0.0, 0.0}) {
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    }

    virtual void move() = 0;
    virtual void renderTurtle() = 0;
    virtual ~Turtle() = default;

    bool checkCollision(const Point& other, double otherRadius) {
        double dx = position.x - other.x;
        double dy = position.y - other.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance <= (radius + otherRadius);
    }

    Point getPosition() const { return position; }
    void setPosition(const Point& newPosition) { position = newPosition; }
};

#endif