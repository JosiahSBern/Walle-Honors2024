#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "Point.h"

class Environment {
protected:
    rclcpp::Node::SharedPtr node_;  // ROS2 node
    std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_;
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> teleport_client_;
    Point exit;
    std::string direction;

public:
    Environment(rclcpp::Node::SharedPtr node);
    virtual void drawWalls() = 0;
    virtual void setExit(double x, double y, const std::string& direction);
    Point getExit();
    void drawLine(Point start, Point end);
    void drawRectangle(Point topLeft, Point bottomRight);
    void setPen(bool pen_state, int r = 0, int g = 0, int b = 0, int width = 2);
    void quit();
    virtual ~Environment() = default;
};




// Derived class: ClassroomEnvironment
class ClassroomEnvironment : public Environment {
    private:
        double roomLength;
        double roomWidth;
        double desk_width;
        double desk_height;
        double desk_spacing ;
        int desksPerRow;
        int desksPerColumn;
    public:
        ClassroomEnvironment(rclcpp::Node::SharedPtr node);
        void drawWalls();
        void drawExit();
        void drawDesk();
        void drawClassroom();
        // void spawnStartingTurtle();
};





// class RectangularRoom : public Environment {
// public:
//     RectangularRoom(rclcpp::Node::SharedPtr node);
//     void drawWalls() override;
// };
#endif  // ENVIRONMENT_H

