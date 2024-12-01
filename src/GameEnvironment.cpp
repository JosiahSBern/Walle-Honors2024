#include "GameEnvironment.h"
#include "CollisionHandler.h"
#include <cmath>

// Constructor
GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node) : Environment(node) {}

// Draw bins
void GameEnvironment::drawBins() {
    drawRectangle({1.0, 9.0}, {3.0, 7.5});
    drawRectangle({4.0, 9.0}, {6.0, 7.5});
    drawRectangle({7.0, 9.0}, {9.0, 7.5});
}

// Populate the assortment box
void GameEnvironment::populateAssortment() {
    objects = {{Point{2.0, 2.0}, RECYCLING, "Bottle"}, {Point{3.0, 2.5}, PAPER, "Newspaper"}};
    for (const auto& obj : objects) {
        drawRectangle(obj.position, {obj.position.x + 0.5, obj.position.y + 0.5});
    }
}

void GameEnvironment::handleSorting() {
    CollisionHandler collisionHandler(11.0, 11.0);

    for (auto& obj : objects) {
        for (size_t i = 0; i < binPositions.size(); ++i) {
            const auto& binTopLeft = binPositions[i];
            const auto binBottomRight = Point{binTopLeft.x + 2.0, binTopLeft.y - 1.5};

            // Check if object is within bin
            if (obj.position.x >= binTopLeft.x && obj.position.x <= binBottomRight.x &&
                obj.position.y <= binTopLeft.y && obj.position.y >= binBottomRight.y) {
                if (static_cast<ObjectType>(i) == obj.type) {
                    RCLCPP_INFO(node_->get_logger(), "%s sorted correctly!", obj.name.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), "%s sorted incorrectly!", obj.name.c_str());
                }
            } else {
                obj.position = collisionHandler.adjustPositionWithSoftCollision(obj.position, obj.position);
            }
        }
    }
}

void GameEnvironment::drawGame() {
    drawBins();
    populateAssortment();
}

void GameEnvironment::drawWalls() {
    RCLCPP_INFO(node_->get_logger(), "Drawing walls in the game environment...");

    // Example: Drawing a rectangle as the game boundary
    Point topLeft = {1.0, 1.0};
    Point bottomRight = {10.0, 10.0};

    // Use the drawRectangle method inherited from Environment
    drawRectangle(topLeft, bottomRight);
}