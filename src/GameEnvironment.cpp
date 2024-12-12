// src/GameEnvironment.cpp
//Working
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
    double stepSize = 0.1;         // Small step size for smooth movement
    double followDistance = 1.0;  // Distance for TrashTurtle to follow
    size_t currentTrashTurtleIndex = 0;  // Index of the current TrashTurtle being processed

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

            // Step 1: Move turtle1 to the TrashTurtle's current position
            Point turtle1Position = centralTurtle->getPosition();
            Point trashTurtlePosition = trashTurtle->getPosition();
            double dx = trashTurtlePosition.x - turtle1Position.x;
            double dy = trashTurtlePosition.y - turtle1Position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance > stepSize) {
                // Calculate the next step for smooth movement
                double stepX = turtle1Position.x + stepSize * (dx / distance);
                double stepY = turtle1Position.y + stepSize * (dy / distance);

                // Teleport turtle1 to the next step
                if (teleport_client_->wait_for_service(std::chrono::seconds(1))) {
                    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
                    request->x = stepX;
                    request->y = stepY;
                    request->theta = std::atan2(dy, dx);

                    auto result = teleport_client_->async_send_request(request);
                    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
                        RCLCPP_INFO(node_->get_logger(), "Turtle1 moving towards TrashTurtle %s.", trashTurtle->getName().c_str());
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "Failed to move Turtle1 smoothly.");
                    }
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));  // Slow down for visibility
                continue;
            }

            // Step 2: Move TrashTurtle to its designated bin following turtle1
            TrashType type = trashTurtle->getTrashType();
            Point targetBin = binPositions[static_cast<size_t>(type)];
            dx = targetBin.x - trashTurtlePosition.x;
            dy = targetBin.y - trashTurtlePosition.y;
            distance = std::sqrt(dx * dx + dy * dy);

            if (distance > stepSize) {
                // Move TrashTurtle one step closer to the bin
                trashTurtle->move(*centralTurtle, followDistance);
                rclcpp::sleep_for(std::chrono::milliseconds(100));  // Slow down for visibility
                continue;
            }

            // Step 3: Finalize sorting for the current TrashTurtle
            if (trashTurtle->getCurrentState() == SortState::MOVING_TO_BIN) {
                trashTurtle->setCurrentState(SortState::SORTED);
                trashTurtle->stopAtTarget();
                RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s has been sorted.", trashTurtle->getName().c_str());
            }

            // Move to the next TrashTurtle
            currentTrashTurtleIndex++;
        }

        // Allow other ROS2 processes to run
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // Control update rate
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
    const double binHeight = 3;
    const int LINE_WIDTH = 4;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point binBottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};

        // Set the pen color and draw the bin
        int r, g, b;
        switch (i) {
            case 0: r = 0; g = 100; b = 0; break;       // Green (Trash)
            case 1: r = 0; g = 0; b = 255; break;       // Blue (Recycling)
            case 2: r = 128; g = 128; b = 128; break;   // Gray (Paper)
            default: r = g = b = 0;                    // Default Black
        }

        drawRectangle(binPositions[i], binBottomRight, r, g, b,LINE_WIDTH);
        RCLCPP_INFO(node_->get_logger(), "Bin %zu drawn: TopLeft (%.2f, %.2f), BottomRight (%.2f, %.2f), Color (%d, %d, %d)",
                    i + 1, binPositions[i].x, binPositions[i].y, binBottomRight.x, binBottomRight.y, r, g, b);
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


