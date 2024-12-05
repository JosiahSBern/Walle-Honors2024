#ifndef TRASH_TURTLE_H
#define TRASH_TURTLE_H

#include "Turtle.h"
#include "Point.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

enum class TrashType {
    PLASTIC,
    PAPER,
    ORGANIC
};

class TrashTurtle : public Turtle {
private:
    TrashType type;                               // Type of trash the turtle is responsible for
    Point targetPosition;                         // Target position for the TrashTurtle (e.g., bin location)
    double targetRadius;                          // Radius within which the turtle considers itself at the target
    double followDistanceThreshold_;             // Distance threshold for following the leader turtle
    bool followingLeader_;                        // Whether the TrashTurtle is currently following the leader
    std::shared_ptr<Turtle> leaderTurtle;         // Pointer to the leader (TeleopTurtle)

    void followLeader();                          // Logic for following the leader turtle
    void updateVelocityToTarget(const Point& target); // Updates velocity to move toward a target point

public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
                double radius, TrashType type, const Point& target);

    void setLeaderTurtle(std::shared_ptr<Turtle> leader); // Assign the leader turtle
    void moveToBin();                                     // Move to the assigned bin or follow the leader
    void move() ;                                 // Override to include following behavior
    void renderTurtle();                        // Render the TrashTurtle
    bool isAtTarget() const;                              // Check if TrashTurtle is at its target
    void setTargetPosition(const Point& target);          // Set the target position
    TrashType getTrashType() const;                       // Get the type of trash handled by this turtle
};

#endif // TRASH_TURTLE_H
