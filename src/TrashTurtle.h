#ifndef TRASHTURTLE_H
#define TRASHTURTLE_H
#include "Turtle.h"
#include <memory>

enum class TrashType {
    PLASTIC,
    PAPER,
    ORGANIC
};

class TrashTurtle : public Turtle {
private:
    TrashType type;
    Point targetPosition;
    double targetRadius;
    std::shared_ptr<Turtle> leaderTurtle; // The turtle to follow

public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, 
            double radius, TrashType type, const Point& target);


    void setLeaderTurtle(std::shared_ptr<Turtle> leader);
    void moveToBin();
    void move();
    void renderTurtle();

    void setTargetPosition(const Point& target);
    TrashType getTrashType() const;

private:
    void updateVelocityToTarget(const Point& target);
    bool isAtTarget() const;
};

#endif