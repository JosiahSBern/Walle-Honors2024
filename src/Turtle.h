#ifndef TURTLE_H
#define TURTLE_H

#include "Point.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

class Turtle {
protected:
    Point position;  // Current turtle position
    std::string name;  // Turtle's name
    double radius;  // Radius of the turtle's bounding circle
    double orientation;
    turtlesim::msg::Pose pose;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;  // Teleport service client


public:
    double calculateDistance(const Point& p1, const Point& p2) const;
    Turtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius);
    virtual ~Turtle();
     std::string getName() const { return name; }

    bool checkCollision(const Point& other, double otherRadius) const;

    Point getPosition() const;
    void setPosition(const Point& newPosition);
    void setOrientation(double theta);
    double getOrientation() const;


    void renderTurtle(); 

    turtlesim::msg::Pose getPose() const; // Getter for the full Pose
    void updatePose(); 
    void teleportToPosition(double x, double y, double theta);
};

#endif
