#ifndef TRASH_TURTLE_H
#define TRASH_TURTLE_H

#include "Turtle.h"
#include "Point.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

enum class TrashType {
    PLASTIC,
    PAPER,
    ORGANIC
};

class TrashTurtle : public Turtle {
private:
    TrashType trashType;    // Type of trash this turtle represents
    Point boxPosition;      // Bin position for this turtle
    double speed;           // Movement speed
    Point targetPosition;   // Current target position

    // Update the turtle's velocity to move towards the target position
    void updateVelocityToTarget(const Point& target);

public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, 
                TrashType type, const Point& boxPosition, double speed = 0.5);

    // Implement movement logic
    void move();

    // Set the target position
    void setTargetPosition(Point& target);

    // Render the turtle visually
    void renderTurtle();

    // Move to its designated bin
    void moveToBin();

    // Get the trash type
    TrashType getTrashType();
};

#endif  // TRASH_TURTLE_H
