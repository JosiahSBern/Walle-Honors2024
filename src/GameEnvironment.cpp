#include "GameEnvironment.h"
#include "TrashTurtle.h"
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/spawn.hpp"
#include <memory>

GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name)
    : Environment(node, turtle_name) {
    // Initialize bin positions
    const double leftWallOffset = 1.5;
    const double binSpacing = 3.0;
    const double binTopY = 9.0;

    binPositions = {
        {leftWallOffset, binTopY},
        {leftWallOffset + binSpacing, binTopY},
        {leftWallOffset + 2 * binSpacing, binTopY}
    };

    // Initialize leader turtle (replace with actual initialization logic)
    turtle1 = std::make_shared<Turtle>(node, "LeaderTurtle", 1.0);
}

void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing game...");
    drawWalls();
    drawBins();
    spawnTrashTurtles();
    updateTrashTurtles();
}

void GameEnvironment::spawnTrashTurtles() {
    RCLCPP_INFO(node_->get_logger(), "Spawning TrashTurtles...");

    trashTurtles.clear(); // Clear any existing turtles

    // Constants for offsets
    const double offsetX = 0.3;  // Horizontal offset for the second turtle
    const double offsetY = 0.0;  // No vertical offset

    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point center = binPositions[i];
        center.x += 1.0;  // Move to the center of the bin (assuming bin width is 2.0)

        // Spawn first turtle in the middle of the box
        std::string name1 = "Trash" + std::to_string(i * 2 + 1);
        spawnTurtle(name1, center.x, center.y, 0.0);

        // Spawn second turtle with a slight horizontal offset
        std::string name2 = "Trash" + std::to_string(i * 2 + 2);
        spawnTurtle(name2, center.x + offsetX, center.y + offsetY, 0.0);

        // Add the spawned turtles to the trashTurtles list
        auto trashTurtle1 = std::make_shared<TrashTurtle>(node_, name1, 0.5, static_cast<TrashType>(i), center);
        auto trashTurtle2 = std::make_shared<TrashTurtle>(node_, name2, 0.5, static_cast<TrashType>(i), Point{center.x + offsetX, center.y + offsetY});
        
        trashTurtles.push_back(trashTurtle1);
        trashTurtles.push_back(trashTurtle2);

        RCLCPP_INFO(node_->get_logger(), "Spawned turtles %s and %s for bin %zu", name1.c_str(), name2.c_str(), i + 1);
    }
}



void GameEnvironment::updateTrashTurtles() {
    RCLCPP_INFO(node_->get_logger(), "Updating TrashTurtles...");
    for (auto& turtle : trashTurtles) {
        turtle->move();
        turtle->renderTurtle();
    }
}

void GameEnvironment::drawBins() {
    const double binWidth = 2.0;
    const double binHeight = 1.5;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point binBottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};

        // Set the pen color and draw the bin
        int r, g, b;
        switch (i) {
            case 0: r = 0; g = 255; b = 0; break;       // Green (Trash)
            case 1: r = 0; g = 0; b = 255; break;       // Blue (Recycling)
            case 2: r = 128; g = 128; b = 128; break;   // Gray (Paper)
            default: r = g = b = 0;                    // Default Black
        }

        drawRectangle(binPositions[i], binBottomRight, r, g, b);
        RCLCPP_INFO(node_->get_logger(), "Bin %zu drawn: TopLeft (%f, %f), BottomRight (%f, %f), Color (%d, %d, %d)",
                    i + 1, binPositions[i].x, binPositions[i].y, binBottomRight.x, binBottomRight.y, r, g, b);
    }

    // Draw the bottom box
    Point bottomBoxTopLeft = {1.0, 3.0};
    Point bottomBoxBottomRight = {10.0, 1.0};
    drawRectangle(bottomBoxTopLeft, bottomBoxBottomRight, 255, 255, 255); // White
    RCLCPP_INFO(node_->get_logger(), "Bottom box drawn: TopLeft (%f, %f), BottomRight (%f, %f), Color (255, 255, 255)",
                bottomBoxTopLeft.x, bottomBoxTopLeft.y, bottomBoxBottomRight.x, bottomBoxBottomRight.y);
}

void GameEnvironment::drawWalls() {
    const double WALL_LEFT = 1.0;
    const double WALL_RIGHT = 10.0;
    const double WALL_TOP = 10.0;
    const double WALL_BOTTOM = 1.0;

    Point topLeft = {WALL_LEFT, WALL_TOP};
    Point bottomRight = {WALL_RIGHT, WALL_BOTTOM};
    drawRectangle(topLeft, bottomRight, 255, 255, 255); // White

    RCLCPP_INFO(node_->get_logger(), "Walls drawn: TopLeft (%f, %f), BottomRight (%f, %f)",
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
}