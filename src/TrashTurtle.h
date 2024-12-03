#ifndef TRASH_TURTLE_H
#define TRASH_TURTLE_H

#include "Turtle.h"
#include "Point.h"

enum TrashType {
    PLASTIC,
    PAPER,
    ORGANIC
};

class TrashTurtle : public Turtle {
private:
    TrashType type;
    Point targetPosition;
    double targetRadius;

public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius, TrashType type, const Point& target, double targetRadius);

    void updateVelocityToTarget(const Point& target);
    void move() override;
    void renderTurtle() override;
    void setTargetPosition(const Point& target);
    TrashType getTrashType() const;
};

#endif  // TRASH_TURTLE_H
