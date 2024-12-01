#ifndef COLLISION_HANDLER_H
#define COLLISION_HANDLER_H

#include "Point.h"
#include <vector>

class CollisionHandler {
public:
    CollisionHandler(double areaWidth, double areaHeight);

    // Check if a point is within the defined bounds
    bool isWithinBounds(const Point& position) const;

    // Check for collision between two circular objects with radii
    bool checkCollision(const Point& p1, double r1, const Point& p2, double r2) const;

    // Adjust position to keep it within bounds
    Point adjustPositionWithSoftCollision(const Point& current, const Point& proposed) const;

private:
    double areaWidth_;  // Width of the bounded area
    double areaHeight_; // Height of the bounded area
};

#endif
