#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <std_srvs/srv/empty.hpp>  s
#include "Point.h"

class Environment {
protected:
    std::string turtle_name_;  // Name of the turtle
    rclcpp::Node::SharedPtr node_;  // ROS2 node for managing environment
    std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_;
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> teleport_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;  // Client for /clear service


    Point exit;
    std::string direction;

public:
    Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    virtual void drawWalls() = 0;
    void clearEnvironment(); 
    virtual void setExit(double x, double y, const std::string& direction);
    Point getExit();
    void drawLine(Point start, Point end, bool pen_state, int r, int g, int b, int width)    void drawRectangle(Point topLeft, Point bottomRight);
    void setPen(bool pen_state, int r, int g , int b, int width = 2);
    void quit();
    virtual ~Environment() = default;
};

#endif  // ENVIRONMENT_H
