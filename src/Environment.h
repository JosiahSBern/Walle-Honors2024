#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/set_pen.hpp"  //Set Pen color and Width
#include "turtlesim/srv/teleport_absolute.hpp"//Teleport Turtle
#include "turtlesim/srv/spawn.hpp" //Spawn new Turtles
#include "Point.h"

//Enivronment is the base class for managing turtles in a simulated enivronment.
class Environment {
protected:
    std::string turtle_name_;
    std::string direction;  

    /*Built in TurtleSim Clients*/
    rclcpp::Node::SharedPtr node_;  
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;  
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;  
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_; 
public:
    //Deconstructor
    virtual ~Environment() = default;

    //Constructor
    Environment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);

    //Spawn a new turtle at a specified position and orientation
    void spawnTurtle(const std::string& name, double x, double y, double theta);

    //Draw a colored line between two points
    void drawLine(Point start, Point end, bool pen_state, int r, int g, int b, int width);

    //Draw a rectangle given the top-left and bottom-right corners
    void drawRectangle(Point topLeft, Point bottomRight, int r, int g, int b);

     // Set the pen state, color, and width
    void setPen(bool pen_state, int r, int g, int b, int width = 2);

    //Quit and Cleanup
    void quit();
};

#endif
