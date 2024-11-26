#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <memory>  // For std::shared_ptr
#include <rclcpp/rclcpp.hpp>  // ROS2 header for rclcpp
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

using namespace std;
struct Point {
    double x;
    double y;
};

class Environment {
protected:
    rclcpp::Node::SharedPtr node_;  // ROS2 node
    std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_;  // Set pen client
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> teleport_client_;  // Teleport client
    // rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    Point exit;  // Exit point
    string direction;  // Direction of exit
public:
    // Constructor
    Environment(rclcpp::Node::SharedPtr node);

    // Virtual drawWalls function
    virtual void drawWalls() = 0;

    // Virtual setExit function
    virtual void setExit(double x, double y, const std::string& direction);

    // Get exit position
    Point getExit();

    // Draw line between two points
    void drawLine(Point start, Point end);
    void drawRectangle(Point topLeft, Point bottomRight); 

    // Function to set pen on or off
    void setPen(bool on,int r, int g, int b);
    // Quit function to end the drawing session
    void quit();
};

// Derived class: ClassroomEnvironment
class ClassroomEnvironment : public Environment {
    private:
        double roomLength;
        double roomWidth;
        double desk_width;
        double desk_height;
        double desk_spacing ;
    public:
        ClassroomEnvironment(rclcpp::Node::SharedPtr node);
        void drawWalls();
        void drawExit();
        void drawDesk();
        void drawClassroom();
};





// class RectangularRoom : public Environment {
// public:
//     RectangularRoom(rclcpp::Node::SharedPtr node);
//     void drawWalls() override;
// };
#endif  // ENVIRONMENT_H

