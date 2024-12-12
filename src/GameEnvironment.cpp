// src/GameEnvironment.cpp
//kk
#include "GameEnvironment.h"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include <memory>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "turtlesim/msg/pose.hpp"
#include "Point.h"

GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& central_turtle_name)
    : Environment(node, central_turtle_name) 
{
    this->spawn_client_ = this->node_->create_client<turtlesim::srv::Spawn>("/spawn");
    this->kill_client_ = this->node_->create_client<turtlesim::srv::Kill>("/kill");

    const double leftWallOffset = 1.5;
    const double binSpacing = 3.0;
    const double binTopY = 9.0;

    this->binPositions = {
        {leftWallOffset, binTopY},
        {leftWallOffset + binSpacing, binTopY},
        {leftWallOffset + 2 * binSpacing, binTopY}
    };

    //Initialize centralTurtle as turtle1
    this->centralTurtle = std::make_shared<Turtle>(node, "turtle1", 0.5);
}


GameEnvironment::~GameEnvironment() {
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    trashTurtles.clear();
}

void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing game...");
    initializeEnvironment();

    //Draw static elements
    drawWalls();
    drawBins();

    //Spawn initial TrashTurtles
    spawnTrashTurtles();
    RCLCPP_INFO(node_->get_logger(), "Welcome to the Trash Sorting Game!");
    RCLCPP_INFO(node_->get_logger(), "Use the arrow keys to move the central turtle.");
    RCLCPP_INFO(node_->get_logger(), "Your goal: Help the TrashTurtles reach their designated bins.");
    RCLCPP_INFO(node_->get_logger(), "Trash Types: Green (Trash), Blue (Recycling), Gray (Paper).");
     RCLCPP_INFO(node_->get_logger(), "Press Enter to start.");
    std::cin.get();


    updateTrashTurtles(); 
}


void GameEnvironment::initializeEnvironment() {
    auto pose_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) {
        centralTurtle->setPosition({msg->x, msg->y});
        centralTurtle->setOrientation(msg->theta);  // Update turtle1's orientation
        RCLCPP_INFO(node_->get_logger(), "Turtle1 Pose Updated: x=%.2f, y=%.2f, theta=%.2f", msg->x, msg->y, msg->theta);
    };

    node_->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, pose_callback);
}



void GameEnvironment::spawnTrashTurtles() {
    RCLCPP_INFO(node_->get_logger(), "Spawning TrashTurtles...");

    const double bottomBoxLeft = 1.0;
    const double bottomBoxRight = 10.0;
    const double bottomBoxTop = 3.0;
    const double bottomBoxBottom = 1.0;

    const int numTurtles = 4;
    const double horizontalSpacing = (bottomBoxRight - bottomBoxLeft) / (numTurtles + 1);
    for (size_t i = 0; i < numTurtles; ++i) {
        TrashType type = static_cast<TrashType>(i % 3);  //Cycle through trash types
        std::string name = "Trash" + std::to_string(i + 1);

        double xPos = bottomBoxLeft + (i + 1) * horizontalSpacing;
        double yPos = (bottomBoxTop + bottomBoxBottom) / 2;

        //Spawn the turtle in the simulation
        if (spawn_client_->wait_for_service(std::chrono::seconds(5))) {
            auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
            spawn_request->x = xPos;
            spawn_request->y = yPos;
            spawn_request->theta = 0.0;
            spawn_request->name = name;

            auto spawn_result = spawn_client_->async_send_request(spawn_request);
            if (rclcpp::spin_until_future_complete(node_, spawn_result) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to spawn %s.", name.c_str());
                continue;
            } else {
                RCLCPP_INFO(node_->get_logger(), "Successfully spawned %s at (%.2f, %.2f).", name.c_str(), xPos, yPos);
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Spawn service not available. Cannot spawn %s.", name.c_str());
            continue;
        }

        // Instantiate the TrashTurtle object
        auto trashTurtle = std::make_shared<TrashTurtle>(
            node_,
            name,
            0.5,
            type,
            binPositions[i % binPositions.size()]
        );

        trashTurtle->setPosition({xPos, yPos});
        trashTurtles.push_back(trashTurtle);
        trashTurtles[i]->setCurrentState(SortState::MOVING_TO_BIN);
    }
}
    




void GameEnvironment::updateTrashTurtles() {
    double stepSize = 0.2;         // Larger step size for faster movement
    double followDistance = 1.0;  // Distance for TrashTurtle to follow
    size_t currentTrashTurtleIndex = 0;  // Index of the current TrashTurtle being processed
    bool movingToTrashTurtle = true;     // Whether turtle1 is moving to the TrashTurtle's position

    while (rclcpp::ok()) {
        // Check if all TrashTurtles are sorted
        bool allSorted = true;
        for (const auto& trashTurtle : trashTurtles) {
            if (trashTurtle->getCurrentState() != SortState::SORTED) {
                allSorted = false;
                break;
            }
        }

        // If all TrashTurtles are sorted, end the program
        if (allSorted) {
            RCLCPP_INFO(node_->get_logger(), "All TrashTurtles are sorted. Ending the game!");
            rclcpp::shutdown();
            return;
        }

        // Ensure we have a valid TrashTurtle to process
        if (currentTrashTurtleIndex < trashTurtles.size()) {
            auto& trashTurtle = trashTurtles[currentTrashTurtleIndex];

            if (movingToTrashTurtle) {
                // Move turtle1 to the TrashTurtle's position
                Point turtle1Position = centralTurtle->getPosition();
                Point trashTurtlePosition = trashTurtle->getPosition();
                double dx = trashTurtlePosition.x - turtle1Position.x;
                double dy = trashTurtlePosition.y - turtle1Position.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                if (distance > stepSize) {
                    // Teleport turtle1 to the TrashTurtle
                    centralTurtle->teleportToPosition(trashTurtlePosition.x, trashTurtlePosition.y, std::atan2(dy, dx));
                    RCLCPP_INFO(node_->get_logger(), "Turtle1 moved to TrashTurtle %s.", trashTurtle->getName().c_str());
                }

                movingToTrashTurtle = false;  // Switch to moving both turtles to the bin
            } else {
                // Move both turtle1 and TrashTurtle to the bin
                TrashType type = trashTurtle->getTrashType();
                Point targetBin = binPositions[static_cast<size_t>(type)];
                Point turtle1Position = centralTurtle->getPosition();
                Point trashTurtlePosition = trashTurtle->getPosition();

                // Teleport turtle1 directly to the center of the bin
                centralTurtle->teleportToPosition(targetBin.x, targetBin.y, 0.0);

                // Move TrashTurtle towards the bin, following turtle1
                trashTurtle->move(*centralTurtle, followDistance);

                // Check if TrashTurtle reached the bin
                double dx = targetBin.x - trashTurtlePosition.x;
                double dy = targetBin.y - trashTurtlePosition.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                if (distance <= stepSize) {
                    // Finalize sorting for the current TrashTurtle
                    trashTurtle->setCurrentState(SortState::SORTED);
                    trashTurtle->stopAtTarget();
                    RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s has been sorted.", trashTurtle->getName().c_str());
                    movingToTrashTurtle = true;  // Reset for the next TrashTurtle
                    currentTrashTurtleIndex++;
                }
            }
        }

        // Allow other ROS2 processes to run
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(50));  // Faster update rate
    }
}





void GameEnvironment::drawWalls() {
    const double WALL_LEFT = 1.0;
    const double WALL_RIGHT = 10.0;
    const double WALL_TOP = 10.0;
    const double WALL_BOTTOM = 1.0;
    const int LINE_WIDTH = 6;

    Point topLeft = {WALL_LEFT, WALL_TOP};
    Point bottomRight = {WALL_RIGHT, WALL_BOTTOM};
    drawRectangle(topLeft, bottomRight, 255, 255, 255,LINE_WIDTH); // White

    RCLCPP_INFO(node_->get_logger(), "Walls drawn: TopLeft (%.2f, %.2f), BottomRight (%.2f, %.2f)",
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
}

void GameEnvironment::drawBins() {
    const double binWidth = 2.0;
    const double binHeight = 3.0;
    const int LINE_WIDTH = 4;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point topLeft = binPositions[i];
        Point bottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};

        // Calculate the bin center dynamically
        double centerX = (topLeft.x + bottomRight.x) / 2.0;
        double centerY = (topLeft.y + bottomRight.y) / 2.0;

        // Store the center as the target bin position for TrashTurtles
        binPositions[i] = {centerX, centerY};

        // Draw the bin as a rectangle
        drawRectangle(topLeft, bottomRight, 0, 255, 255, LINE_WIDTH);  // Cyan for bins
        RCLCPP_INFO(node_->get_logger(), "Bin %zu drawn: TopLeft (%.2f, %.2f), BottomRight (%.2f, %.2f)",
                    i + 1, topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    }
}




void GameEnvironment::moveTurtleToBins() {
    RCLCPP_INFO(node_->get_logger(), "Turtle1 starting to visit all bins...");

    for (const auto& binPosition : binPositions) {
        RCLCPP_INFO(node_->get_logger(), "Moving Turtle1 to bin at (%.2f, %.2f)...", binPosition.x, binPosition.y);

        // Teleport turtle1 to the bin position
        if (teleport_client_->wait_for_service(std::chrono::seconds(1))) {
            auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            request->x = binPosition.x;
            request->y = binPosition.y;
            request->theta = 0.0;  // Set orientation (optional)

            auto result = teleport_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Turtle1 successfully moved to bin at (%.2f, %.2f).", binPosition.x, binPosition.y);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to move Turtle1 to bin at (%.2f, %.2f).", binPosition.x, binPosition.y);
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "TeleportAbsolute service not available for Turtle1.");
        }

        // Add a small delay for better visibility
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(node_->get_logger(), "Turtle1 has visited all bins.");
}


