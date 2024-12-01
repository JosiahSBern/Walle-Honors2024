#include "CollisionHandler.h"
#include <cmath>

// Constructor to set boundary dimensions
CollisionHandler::CollisionHandler(double areaWidth, double areaHeight)
    : areaWidth_(areaWidth), areaHeight_(areaHeight) {}

// Check if a position is within bounds
bool CollisionHandler::isWithinBounds(const Point& position) const {
    // Returns true if the position is within the defined boundaries
    return position.x >= 0 && position.x <= areaWidth_ &&
           position.y >= 0 && position.y <= areaHeight_;
}

// Check for collision between two circular objects
bool CollisionHandler::checkCollision(const Point& p1, double r1, const Point& p2, double r2) const {
    // Calculate the Euclidean distance between two points
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Return true if the distance is less than or equal to the combined radii
    return distance <= (r1 + r2);
}

// Adjust position with soft collision handling
Point CollisionHandler::adjustPositionWithSoftCollision(const Point& proposed) const {
    Point adjusted = proposed;

    // Boundary checks: Prevent the position from going outside defined boundaries
    if (adjusted.x < 0) adjusted.x = 0;
    if (adjusted.x > areaWidth_) adjusted.x = areaWidth_;
    if (adjusted.y < 0) adjusted.y = 0;
    if (adjusted.y > areaHeight_) adjusted.y = areaHeight_;

    return adjusted;
}
