#ifndef TRASHTURTLE_H
#define TRASHTURTLE_H

#include "Turtle.h"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include "TrashType.h"
#include "Point.h"

// Enum for Sorting States
enum class SortState {
    MOVING_TO_BIN,
    SORTED
};

class TrashTurtle : public Turtle {
public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type = TrashType::NONE, const Point& target = {0.0, 0.0});
    
    bool isAtTarget() const; // Check if the turtle is at the target position
    void move(const Turtle& target, double follow_distance); // Move the trash turtle
    void updateVelocityToTarget(const Point& target); // Update velocity towards the target
    void stopMovement(); // Stop the movement of the turtle

    void setTargetPosition(const Point& target); // Set the target position for the turtle
    TrashType getTrashType() const; // Get the type of trash
    void handleSorting(); // Handle the sorting of the trash into bins
    void setPenColor(int r, int g, int b, int width); // Set the pen color for the turtle

    // New function to set the pen color based on the TrashType
    void setPenColorForTrashType();

    Point getBinPositionForTrashType() const; // Get the corresponding bin position for the trash type
    SortState getCurrentState() const; // Get the current sorting state
    double calculateDistance(const Point& p1, const Point& p2) const;
    void teleportToBinCenter() ;
    void stopAtTarget();


private:
    TrashType type; // Type of trash (Trash, Recycling, Paper)
    SortState currentState_; // Current sorting state
    Point targetPosition; // Position of the target bin
    double targetRadius; // Radius to check if at the target
    std::shared_ptr<rclcpp::Client<turtlesim::srv::SetPen>> pen_client_; // Client to control pen color
    std::shared_ptr<rclcpp::Client<turtlesim::srv::TeleportAbsolute>> teleport_client_; // Teleport client
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> twist_pub_; // Publisher for velocity commands
    void setCurrentState(SortState newState);
};

#endif // TRASHTURTLE_H
