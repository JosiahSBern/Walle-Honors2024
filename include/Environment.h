#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
using namespace std;


struct Point{
    double x;
    double y;
};

class Environment
{
    protected:
        rclcpp::Node::SharedPtr node_; //Ros2 node
        shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_; //Set pen client
        Point exit; //Exit point
        string direction; //Direction of exit
    public:
        //Constructor
        Environment(rclcpp::Node::SharedPtr node);

        //Virtual drawWalls function
        virtual void drawWalls() = 0;

        //Virtual setExit function
        virtual void setExit(double x, double y, const string& direction);

        //Get exit position
        Point getExitPosition();

        //Draw line between two points
        void drawLine(Point start, Point end);

        //Function to set pen on or off
        void setPen(bool on);

};



class RectangularRoom : public Environment {
public:
    RectangularRoom(rclcpp::Node::SharedPtr node);
    void drawWalls() override;
};

