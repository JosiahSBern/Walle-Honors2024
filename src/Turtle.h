#ifndef TURTLE_H
#define TURTLE_H

#include <string>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "Point.h"

class Turtle {
public:
    Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius);
    virtual ~Turtle();

    double calculateDistance(const Point& p1, const Point& p2) const;
    bool checkCollision(const Point& other, double otherRadius) const;

    Point getPosition() const;
    void setPosition(const Point& newPosition);
    std::string getName() const;
    


protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::string name;
    double radius;
    Point position;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

#endif // TURTLE_H
