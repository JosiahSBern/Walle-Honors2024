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
    const double binWidth = 2.0;
    const double binHeight = 1.5;
    const double bottomBoxHeight = 2.0;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");

    // Draw top bins
    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point binBottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};
        drawRectangle(binPositions[i], binBottomRight);
        RCLCPP_INFO(node_->get_logger(), "Bin %zu: TopLeft (%f, %f), BottomRight (%f, %f)", 
                    i + 1, binPositions[i].x, binPositions[i].y, binBottomRight.x, binBottomRight.y);
    }

    // Draw the large box at the bottom
    Point bottomBoxTopLeft = {1.0, 3.0};
    Point bottomBoxBottomRight = {10.0, 3.0 - bottomBoxHeight};
    drawRectangle(bottomBoxTopLeft, bottomBoxBottomRight);

    RCLCPP_INFO(node_->get_logger(), "Bottom box: TopLeft (%f, %f), BottomRight (%f, %f)",
                bottomBoxTopLeft.x, bottomBoxTopLeft.y, bottomBoxBottomRight.x, bottomBoxBottomRight.y);
}

// Draw the game
void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing the game environment...");
    drawWalls();
    drawBins();
    quit();
}

// Draw walls
void GameEnvironment::drawWalls() {
    const double WALL_LEFT = 1.0;
    const double WALL_RIGHT = 10.0;
    const double WALL_TOP = 10.0;
    const double WALL_BOTTOM = 1.0;

    Point topLeft = {WALL_LEFT, WALL_TOP};
    Point bottomRight = {WALL_RIGHT, WALL_BOTTOM};
    drawRectangle(topLeft, bottomRight);

    RCLCPP_INFO(node_->get_logger(), "Walls drawn: TopLeft (%f, %f), BottomRight (%f, %f)",
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
}
