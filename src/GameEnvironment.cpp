#include "GameEnvironment.h"
#include "CollisionHandler.h"
#include "Point.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// Constructor
GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name) 
    : Environment(node, turtle_name) {
    const double leftWallOffset = 1.5; // Distance from the left wall
    const double binSpacing = 3.0;    // Spacing between bins
    const double binTopY = 9.0;       // Fixed Y-coordinate for bins

    binPositions = {
        {leftWallOffset, binTopY},                   // First bin
        {leftWallOffset + binSpacing, binTopY},      // Second bin
        {leftWallOffset + 2 * binSpacing, binTopY}   // Third bin
    };
}
void GameEnvironment::drawBins() {
    const double binWidth = 2.0;
    const double binHeight = 1.5;
    const double bottomBoxHeight = 2.0;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");

    if (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Pen service not available.");
        return;
    }

    // Draw bins with individual colors
    for (size_t i = 0; i < binPositions.size(); ++i) {
        // Set pen color
        switch (i) {
            case 0:  // Trash (Green)
                setPen(true, 0, 255, 0, 2);
                break;
            case 1:  // Recycling (Blue)
                setPen(true, 0, 0, 255, 2);
                break;
            case 2:  // Paper (Gray)
                setPen(true, 128, 128, 128, 2);
                break;
            default:
                setPen(true, 0, 0, 0, 2); // Default Black
        }

        // Delay to ensure pen updates
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Draw the bin
        Point binBottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};
        drawRectangle(binPositions[i], binBottomRight);

        RCLCPP_INFO(node_->get_logger(), "Bin %zu drawn with color: (%d, %d, %d)", 
                    i + 1, (i == 0 ? 0 : (i == 1 ? 0 : 128)), 
                    (i == 0 ? 255 : (i == 1 ? 0 : 128)), 
                    (i == 0 ? 0 : (i == 1 ? 255 : 128)));
    }

    // Draw the bottom box
    setPen(true, 255, 255, 255, 2); // White
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    Point bottomBoxTopLeft = {1.0, 3.0};
    Point bottomBoxBottomRight = {10.0, 3.0 - bottomBoxHeight};
    drawRectangle(bottomBoxTopLeft, bottomBoxBottomRight);

    RCLCPP_INFO(node_->get_logger(), "Bottom box drawn with color: (255, 255, 255)");
}



// Draw the game
void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing the game environment...");
    drawWalls();
    drawBins();
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
