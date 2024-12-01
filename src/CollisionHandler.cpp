#include "CollisionHandler.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>

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

// Log position adjustments (helper function)
void logAdjustment(const rclcpp::Logger &logger, const Point &original, const Point &adjusted) {
    RCLCPP_INFO(logger, "Adjusting position: Original (%f, %f), Adjusted (%f, %f)",
                original.x, original.y, adjusted.x, adjusted.y);
}

// Adjust position with soft collision handling
Point CollisionHandler::adjustPositionWithSoftCollision(const Point& current, const Point& proposed) const {
    Point adjusted = proposed;

    // Adjust based on boundaries
    if (adjusted.x < 0) adjusted.x = 0;
    if (adjusted.x > areaWidth_) adjusted.x = areaWidth_;
    if (adjusted.y < 0) adjusted.y = 0;
    if (adjusted.y > areaHeight_) adjusted.y = areaHeight_;

    // Log the adjustment for debugging
    logAdjustment(rclcpp::get_logger("CollisionHandler"), proposed, adjusted);

    return adjusted;
}
