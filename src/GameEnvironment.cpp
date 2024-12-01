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

// Draw the game
void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing the game environment...");
    drawWalls();
    drawBins();
}

// Draw walls
void GameEnvironment::drawWalls() {
    Point topLeft = {1.0, 1.0};
    Point bottomRight = {10.0, 10.0};
    drawRectangle(topLeft, bottomRight);
}
