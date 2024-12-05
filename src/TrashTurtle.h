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
    TrashType type;                             
    Point targetPosition;//Target position for the TrashTurtle
    double targetRadius;//Radius within which the turtle considers itself at the target
    double followDistanceThreshold_;//Distance threshold for following the leader turtle
    bool followingLeader_;//Whether the TrashTurtle is currently following the leader
    std::shared_ptr<Turtle> leaderTurtle;//Pointer to the leader Turtle

    void followLeader();//Logic for following the leader turtle
    void updateVelocityToTarget(const Point& target);//Updates velocity

public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
                double radius, TrashType type, const Point& target);

    void setLeaderTurtle(std::shared_ptr<Turtle> leader); //Assign the leader turtle
    void moveToBin(); //Move to the assigned bin or follow the leader
    void move() ;                                
    void renderTurtle();// Render the TrashTurtle
    bool isAtTarget() const;//Check if TrashTurtle is at its target
    void setTargetPosition(const Point& target);//Set the target position
    TrashType getTrashType() const; //Get the type of trash handled by this turtle
};

#endif // TRASH_TURTLE_H
