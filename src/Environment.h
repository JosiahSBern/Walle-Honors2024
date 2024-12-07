#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "Point.h"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/spawn.hpp"

class Environment {
protected:
    std::string turtle_name;
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;

    // Declare twist_pub_ here
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

public:
    Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    virtual ~Environment() = default;

    void drawLine(Point start, Point end, bool pen_state, int r, int g, int b, int width);
    void setPen(bool pen_state, int r, int g, int b, int width = 2);
    void drawRectangle(Point topLeft, Point bottomRight, int r, int g, int b);
    void spawnTurtle(const std::string& name, double x, double y, double theta);
};

#endif // ENVIRONMENT_H
