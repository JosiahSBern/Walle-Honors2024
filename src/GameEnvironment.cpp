#include "GameEnvironment.h"
#include "TrashTurtle.h"
#include <rclcpp/rclcpp.hpp>
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
    trashTurtles.clear();

    trashTurtles.emplace_back(std::make_shared<TrashTurtle>(node_, "Trash1", 0.5, PLASTIC, binPositions[0]));
    trashTurtles.emplace_back(std::make_shared<TrashTurtle>(node_, "Trash2", 0.5, PAPER, binPositions[1]));
    trashTurtles.emplace_back(std::make_shared<TrashTurtle>(node_, "Trash3", 0.5, ORGANIC, binPositions[2]));

    for (auto& turtle : trashTurtles) {
        turtle->setLeaderTurtle(turtle1);
    }
}

void GameEnvironment::updateTrashTurtles() {
    RCLCPP_INFO(node_->get_logger(),
