#include "GameEnvironment.h"
#include "CollisionHandler.h"
#include "Point.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// Constructor
GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node) : Environment(node) {
    const double leftWallOffset = 1.5; // Distance from the left wall
    const double binSpacing = 3.0;    // Spacing between bins
    const double binTopY = 9.0;       // Fixed Y-coordinate for bins

    binPositions = {
        {leftWallOffset, binTopY},                   // First bin
        {leftWallOffset + binSpacing, binTopY},      // Second bin
        {leftWallOffset + 2 * binSpacing, binTopY}   // Third bin
    };
}

// Draw bins
void GameEnvironment::drawBins() {
    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    drawRectangle(binPositions[0], {3.0, 7.5});
    drawRectangle(binPositions[1], {6.0, 7.5});
    drawRectangle(binPositions[2], {9.0, 7.5});
}



// Handle sorting
void GameEnvironment::handleSorting() {
    RCLCPP_INFO(node_->get_logger(), "Handling object sorting...");
    CollisionHandler collisionHandler(11.0, 11.0);

    int correctlySorted = 0, incorrectlySorted = 0;

    for (auto& obj : objects) {
        for (size_t i = 0; i < binPositions.size(); ++i) {
            const auto& binTopLeft = binPositions[i];
            const auto binBottomRight = Point{binTopLeft.x + 2.0, binTopLeft.y - 1.5};

            // Check if object is within bin
            if (obj.position.x >= binTopLeft.x && obj.position.x <= binBottomRight.x &&
                obj.position.y <= binTopLeft.y && obj.position.y >= binBottomRight.y) {
                if (static_cast<ObjectType>(i) == obj.type) {
                    correctlySorted++;
                    RCLCPP_INFO(node_->get_logger(), "%s sorted correctly!", obj.name.c_str());
                } else {
                    incorrectlySorted++;
                    RCLCPP_WARN(node_->get_logger(), "%s sorted incorrectly!", obj.name.c_str());
                }
            } else {
                obj.position = collisionHandler.adjustPositionWithSoftCollision(obj.position, obj.position);
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Sorting complete: %d correctly sorted, %d incorrectly sorted.",
                correctlySorted, incorrectlySorted);
}

// Draw the game
void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing the game environment...");
    drawWalls();
    drawBins();
}

// Draw walls
void GameEnvironment::drawWalls() {
    RCLCPP_INFO(node_->get_logger(), "Drawing walls...");
    Point topLeft = {1.0, 1.0};
    Point bottomRight = {10.0, 10.0};
    drawRectangle(topLeft, bottomRight);
}
