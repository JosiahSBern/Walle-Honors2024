#include "GameEnvironment.h"
#include "CollisionHandler.h"
#include "Point.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// Constructor
GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node) : Environment(node) {
    binPositions = {
        {1.0, 9.0}, // Top-left corner of the first bin
        {4.0, 9.0}, // Top-left corner of the second bin
        {7.0, 9.0}  // Top-left corner of the third bin
    };
}

// Draw bins
void GameEnvironment::drawBins() {
    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    drawRectangle(binPositions[0], {3.0, 7.5});
    drawRectangle(binPositions[1], {6.0, 7.5});
    drawRectangle(binPositions[2], {9.0, 7.5});
}

// Populate the assortment box
void GameEnvironment::populateAssortment() {
    RCLCPP_INFO(node_->get_logger(), "Populating assortment box...");
    const double OBJECT_WIDTH = 0.5;
    const double OBJECT_HEIGHT = 0.5;

    objects = {
        {Point{2.0, 2.0}, RECYCLING, "Bottle"},
        {Point{3.0, 2.5}, PAPER, "Newspaper"}
    };

    for (const auto& obj : objects) {
        drawRectangle(obj.position, {obj.position.x + OBJECT_WIDTH, obj.position.y + OBJECT_HEIGHT});
    }
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
    populateAssortment();
}

// Draw walls
void GameEnvironment::drawWalls() {
    RCLCPP_INFO(node_->get_logger(), "Drawing walls...");
    Point topLeft = {1.0, 1.0};
    Point bottomRight = {10.0, 10.0};
    drawRectangle(topLeft, bottomRight);
}
