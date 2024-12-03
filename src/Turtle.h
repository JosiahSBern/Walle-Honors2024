#ifndef TURTLE_H
#define TURTLE_H

#include "Point.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Turtle {
protected:
    Point position;  // Current turtle position
    std::string name;  // Turtle's name
    double radius;  // Radius of the turtle's bounding circle

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    // Calculate the Euclidean distance between two points
    double calculateDistance(const Point& p1, const Point& p2) const;

public:
    Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius);
    virtual ~Turtle();

    bool checkCollision(const Point& other, double otherRadius) const;

    Point getPosition() const;
    void setPosition(const Point& newPosition);
};

#endif  // TURTLE_H
