#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "Point.h"

// Abstract Base Class: Represents a general environment
class Environment {
protected:
    std::string turtle_name_;  // Name of the turtle
    rclcpp::Node::SharedPtr node_;  // ROS2 node for managing environment
    std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_;  // Pen control client
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> teleport_client_;  // Teleport client
    Point exit;  // Exit point of the environment
    std::string direction;  // Direction of the exit

public:
    // Constructor: Initializes the ROS2 node
    Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    // Draw walls of the environment (pure virtual)
    virtual void drawWalls() = 0;

    // Set the exit point and its direction
    virtual void setExit(double x, double y, const std::string& direction);

    // Get the exit point
    Point getExit();

    // Draw a straight line between two points
    void drawLine(Point start, Point end);

    // Draw a rectangle given top-left and bottom-right points
    void drawRectangle(Point topLeft, Point bottomRight);

    // Set the pen state (on/off) with color and width
    void setPen(bool pen_state, int r = 0, int g = 0, int b = 0, int width = 2);

    // Shut down the environment
    void quit();

    // Virtual destructor for polymorphism
    virtual ~Environment() = default;
};

// Derived Class: Represents a classroom environment
class ClassroomEnvironment : public Environment {
private:
    double roomLength;  // Length of the room
    double roomWidth;   // Width of the room
    double desk_width;  // Width of a single desk
    double desk_height; // Height of a single desk
    double desk_spacing; // Spacing between desks
    int desksPerRow;    // Number of desks per row
    int desksPerColumn; // Number of desks per column

public:
    // Constructor: Initializes a classroom environment
    ClassroomEnvironment(rclcpp::Node::SharedPtr node);

    // Draw walls of the classroom
    void drawWalls();

    // Draw the exit for the classroom
    void drawExit();

    // Draw desks in the classroom
    void drawDesk();

    // Draw the entire classroom
    void drawClassroom();
};

#endif  // ENVIRONMENT_H
