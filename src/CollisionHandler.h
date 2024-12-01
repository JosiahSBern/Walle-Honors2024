#ifndef COLLISION_HANDLER_H
#define COLLISION_HANDLER_H

#include "Point.h"

class CollisionHandler {
public:
    // Constructor to initialize the boundaries
    CollisionHandler(double areaWidth, double areaHeight);

    // Check if a position is within the defined bounds
    bool isWithinBounds(const Point& position) const;

    // Check for collision between two circular objects
    bool checkCollision(const Point& p1, double r1, const Point& p2, double r2) const;

    // Adjust position to keep it within bounds
    Point adjustPositionWithSoftCollision(const Point& proposed) const;

private:
    double areaWidth_;  // Width of the bounded area
    double areaHeight_; // Height of the bounded area
};

#endif  // COLLISION_HANDLER_H
