#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "Point.h"

class Environment {
protected:
    std::string turtle_name_;                        // Name of the turtle
    rclcpp::Node::SharedPtr node_;                   // ROS2 node for managing the environment
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;  // Pen service client
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;  // Teleport service client
    Point exit;
    std::string direction;

public:
    virtual ~Environment() = default;
    Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    virtual void drawWalls() = 0;
    virtual void setExit(double x, double y, const std::string& direction);
    Point getExit();
    void drawLine(Point start, Point end, bool pen_state, int r, int g, int b, int width);
    void drawRectangle(Point topLeft, Point bottomRight, int r, int g, int b);
    void setPen(bool pen_state, int r, int g , int b, int width = 2);
    void quit();
};
#endif 