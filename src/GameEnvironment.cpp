// src/GameEnvironment.cpp
//Work
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
    updateTrashTurtles(); 
}


void GameEnvironment::initializeEnvironment() {
    // Subscribe to /turtle1/pose to track central turtle's position
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

    const int numTurtles = 3;
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
    }
}

void GameEnvironment::updateTrashTurtles() {
    double follow_distance = 1.0; // Desired distance to maintain

    while (rclcpp::ok()) {
        for (auto& trashTurtle : trashTurtles) {
            // Get the real-time position of turtle1
            Point turtle1Position = centralTurtle->getPosition();
            double turtle1Orientation = centralTurtle->getOrientation();

            // Move TrashTurtle to follow turtle1 at the specified distance
            trashTurtle->move(*centralTurtle, follow_distance);
        }

        // Small delay to control the update rate
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


// void GameEnvironment::autoSortTrash() {
//     for (auto& turtle : trashTurtles) {
//         // Check if the TrashTurtle is not sorted yet
//         if (turtle->getCurrentState() != SortState::SORTED) {
//             // Get the type of the TrashTurtle
//             TrashType trashType = turtle->getTrashType();

//             // Get the corresponding bin position based on trash type
//             Point binPosition = getBinPositionForTrashType(trashType);

//             // Move the robot to the bin
//             moveCentralTurtleToBin(binPosition);

//             // After moving to the bin, the robot should "sort" the trash
//             turtle->sortIntoBin();
//         }
//     }
// }





// // Point GameEnvironment::getBinPositionForTrashType(TrashType type) {
//     switch (type) {
//         case TrashType::TRASH:
//             return {1.5, 9.0};  // First bin (green)
//         case TrashType::RECYCLING:
//             return {4.5, 9.0};  // Second bin (blue)
//         case TrashType::PAPER:
//             return {7.5, 9.0};  // Third bin (gray)
//         default:
//             return {5.5, 5.5};  // Default fallback
//     }
// }




void GameEnvironment::handleSorting() {
    for (auto& turtle : trashTurtles) {
        if (turtle->isAtTarget()) {
            RCLCPP_INFO(node_->get_logger(), "TrashTurtle %s is sorted into the correct bin.", turtle->getName().c_str());
        }
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



// void GameEnvironment::moveCentralTurtleToBin(const Point& binPosition) {
//     // Check if the teleport service client is ready
//     if (centralTurtle && teleport_client_->wait_for_service(std::chrono::seconds(1))) {
//         // Get the current position of the central turtle
//         Point currentPos = centralTurtle->getPosition();

//         // Calculate the total distance to the target bin
//         double dx = binPosition.x - currentPos.x;
//         double dy = binPosition.y - currentPos.y;
//         double totalDistance = std::sqrt(dx * dx + dy * dy);

//         // Define the number of steps for the movement (more steps = slower)
//         int numSteps = 10;
//         double stepDistance = totalDistance / numSteps;

//         // Move the turtle in small steps
//         for (int i = 1; i <= numSteps; ++i) {
//             // Calculate the intermediate position for this step
//             double stepX = currentPos.x + (dx * i / numSteps);
//             double stepY = currentPos.y + (dy * i / numSteps);

//             // Create the teleport request for the current step
//             auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
//             request->x = stepX;
//             request->y = stepY;
//             request->theta = 0.0;  // Assuming no rotation needed, can be modified

//             // Send the teleport request and wait for it to complete
//             auto result = teleport_client_->async_send_request(request);
//             if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
//                 RCLCPP_INFO(node_->get_logger(), "Central turtle moved to step %d: (%.2f, %.2f).", i, stepX, stepY);
//             } else {
//                 RCLCPP_ERROR(node_->get_logger(), "Failed to move central turtle to step %d.", i);
//             }

//             // Introduce a small delay to make the movement slower
//             rclcpp::sleep_for(std::chrono::milliseconds(500));  // 500ms delay between each step
//         }

//         // Log the final destination
//         RCLCPP_INFO(node_->get_logger(), "Central turtle reached bin at (%.2f, %.2f).", binPosition.x, binPosition.y);
//     } else {
//         RCLCPP_ERROR(node_->get_logger(), "Teleport service not available for central turtle.");
//     }
// }

