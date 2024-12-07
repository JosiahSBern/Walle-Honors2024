#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>
#include <limits>
#include "GameEnvironment.h"
#include "TeleopTurtle.h"
#include "TrashTurtle.h"
#include "TrashType.h"
#include "Environment.h"
#include "Point.h"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name)
    : Environment(node, "turtle1")  // No semicolon here
{
    // Constructor body starts here
    spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");
    kill_client_ = node_->create_client<turtlesim::srv::Kill>("/kill");

    const double leftWallOffset = 1.5;
    const double binSpacing = 3.0;
    const double binTopY = 9.0;

    binPositions = {
        {leftWallOffset, binTopY},
        {leftWallOffset + binSpacing, binTopY},
        {leftWallOffset + 2 * binSpacing, binTopY}
    };

    teleopTurtle = nullptr;
    activeFollower = nullptr;
    teleopTurtle = std::make_shared<TeleopTurtle>(node_, "turtle1", 0.5);
    teleopTurtle->setPosition({5.5, 5.5});
}

void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing game...");

    // Make sure teleop_turtle exists before drawing
    // After spawning teleop_turtle, give a brief pause to ensure services are available
    // This can help if services are not immediately ready
    // After spawning teleop_turtle
    rclcpp::sleep_for(std::chrono::seconds(3));


    // Now draw since teleop_turtle and its services should exist
    drawWalls();
    drawBins();

    spawnTrashTurtles();

    // Start the update timer
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() {
        RCLCPP_DEBUG(node_->get_logger(), "Updating TrashTurtles...");
        updateTrashTurtles();
    });
}

// void GameEnvironment::initializeEnvironment() {
//     // Kill the default turtle1
//     // if (!kill_client_->wait_for_service(std::chrono::seconds(5))) {
//     //     RCLCPP_ERROR(node_->get_logger(), "Kill service not available.");
//     // } else {
//     //     auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
//     //     kill_request->name = "turtle1";

//     //     auto kill_future = kill_client_->async_send_request(kill_request);
//     //     if (rclcpp::spin_until_future_complete(node_, kill_future) != rclcpp::FutureReturnCode::SUCCESS) {
//     //         RCLCPP_ERROR(node_->get_logger(), "Failed to kill turtle1.");
//     //     } else {
//     //         RCLCPP_INFO(node_->get_logger(), "Successfully removed turtle1.");
//     //     }
//     // }

//     // Spawn the teleop_turtle
//     if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
//         RCLCPP_ERROR(node_->get_logger(), "Spawn service not available.");
//     } else {
//         auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
//         spawn_request->x = 5.5;
//         spawn_request->y = 5.5;
//         spawn_request->theta = 0.0;
//         spawn_request->name = "teleop_turtle";  // Make sure this matches the name used in the Environment constructor

//         auto spawn_future = spawn_client_->async_send_request(spawn_request);
//         if (rclcpp::spin_until_future_complete(node_, spawn_future) != rclcpp::FutureReturnCode::SUCCESS) {
//             RCLCPP_ERROR(node_->get_logger(), "Failed to spawn teleop_turtle.");
//         } else {
//             RCLCPP_INFO(node_->get_logger(), "Successfully spawned teleop_turtle at (5.5, 5.5).");

//             // Instantiate TeleopTurtle with the correct name
//             teleopTurtle = std::make_shared<TeleopTurtle>(node_, "teleop_turtle", 0.5);
//             teleopTurtle->setPosition({5.5, 5.5});
//         }
//     }
// }

void GameEnvironment::spawnTrashTurtles() {
    trashTurtles.clear();

    // Coordinates for where you want to place the trash turtles
    const double bottomBoxLeft = 1.0;
    const double bottomBoxRight = 10.0;
    const double bottomBoxTop = 3.0;
    const double bottomBoxBottom = 1.0;

    const int numTurtles = 9;
    const double horizontalSpacing = (bottomBoxRight - bottomBoxLeft) / (numTurtles + 1);

    // Ensure spawn service is available before proceeding
    if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Spawn service not available. Cannot spawn TrashTurtles.");
        return;
    }

    for (int i = 0; i < numTurtles; ++i) {
        TrashType type = static_cast<TrashType>(i % 3);
        std::string name = "Trash" + std::to_string(i + 1);

        double xPos = bottomBoxLeft + (i + 1) * horizontalSpacing;
        double yPos = (bottomBoxTop + bottomBoxBottom) / 2;

        // 1. Spawn the turtle in turtlesim
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = xPos;
        spawn_request->y = yPos;
        spawn_request->theta = 0.0;
        spawn_request->name = name;

        auto spawn_future = spawn_client_->async_send_request(spawn_request);
        if (rclcpp::spin_until_future_complete(node_, spawn_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle: %s", name.c_str());
            continue; // Skip this turtle if spawn fails
        }

        RCLCPP_INFO(node_->get_logger(), "Spawned %s at (%f, %f)", name.c_str(), xPos, yPos);

        // 2. Now that the turtle exists in turtlesim, create the TrashTurtle object
        auto trashTurtle = std::make_shared<TrashTurtle>(
            node_,
            name,   // Same name as spawned turtle
            0.5,
            type,
            binPositions[i % binPositions.size()]
        );
        
        trashTurtle->setPosition({xPos, yPos});

        // 3. (Optional) Wait a bit or re-check for pen service availability.
        //    This ensures the turtle's services are registered.
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        if (!trashTurtle->waitForPenService(std::chrono::seconds(5))) {
            RCLCPP_WARN(node_->get_logger(), "Pen service not available for %s after waiting.", name.c_str());
        } else {
            // Optionally set a pen color after ensuring service is available
            trashTurtle->setPenColor(255, 255, 0, 3); // Yellow pen, width 3
        }

        trashTurtles.push_back(trashTurtle);
    }

    RCLCPP_INFO(node_->get_logger(), "All TrashTurtles spawned and initialized.");
}


void GameEnvironment::updateTrashTurtles() {
    assignFollower();
    for (auto& turtle : trashTurtles) {
        if (turtle == activeFollower) {
            turtle->followLeader();
        } else {
            turtle->move();
        }
    }
}

void GameEnvironment::resetFollowerPen() {
    if (activeFollower) {
        activeFollower->setPenColor(0, 0, 0, 2);
    }
}

void GameEnvironment::assignFollower() {
    if (!teleopTurtle) {
        RCLCPP_WARN(node_->get_logger(), "TeleopTurtle not initialized yet.");
        return;
    }

    double minDistance = std::numeric_limits<double>::max();
    std::shared_ptr<TrashTurtle> closestTurtle = nullptr;

    for (const auto& trashTurtle : trashTurtles) {
        double distance_to_leader = trashTurtle->calculateDistance(
            trashTurtle->getPosition(),
            teleopTurtle->getPosition()
        );

        if (distance_to_leader < minDistance && distance_to_leader <= 2.0) {
            minDistance = distance_to_leader;
            closestTurtle = trashTurtle;
        }
    }

    if (closestTurtle && activeFollower != closestTurtle) {
        resetFollowerPen();
        activeFollower = closestTurtle;
        activeFollower->setLeaderTurtle(teleopTurtle);
        activeFollower->setPenColor(255, 0, 0, 3); // Red pen
        RCLCPP_INFO(node_->get_logger(), "Assigned %s as follower.", closestTurtle->getName().c_str());
    }
}

void GameEnvironment::drawWalls() {
    const double WALL_LEFT = 1.0;
    const double WALL_RIGHT = 10.0;
    const double WALL_TOP = 10.0;
    const double WALL_BOTTOM = 1.0;

    Point topLeft = {WALL_LEFT, WALL_TOP};
    Point bottomRight = {WALL_RIGHT, WALL_BOTTOM};
    drawRectangle(topLeft, bottomRight, 255, 255, 255);

    RCLCPP_INFO(node_->get_logger(), "Walls drawn: TopLeft (%f, %f), BottomRight (%f, %f)",
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
}

void GameEnvironment::drawBins() {
    const double binWidth = 2.0;
    const double binHeight = 1.5;

    RCLCPP_INFO(node_->get_logger(), "Drawing bins...");
    for (size_t i = 0; i < binPositions.size(); ++i) {
        Point binBottomRight = {binPositions[i].x + binWidth, binPositions[i].y - binHeight};

        int r, g, b;
        switch (i) {
            case 0: r = 0;   g = 255; b = 0;   break; // Green
            case 1: r = 0;   g = 0;   b = 255; break; // Blue
            case 2: r = 128; g = 128; b = 128; break; // Gray
            default: r = g = b = 0;
        }

        drawRectangle(binPositions[i], binBottomRight, r, g, b);
        RCLCPP_INFO(node_->get_logger(), "Bin %zu drawn: TopLeft (%f, %f), BottomRight (%f, %f), Color (%d, %d, %d)",
                    i + 1, binPositions[i].x, binPositions[i].y, binBottomRight.x, binBottomRight.y, r, g, b);
    }
}
GameEnvironment::~GameEnvironment() {
    if (timer_) {
        timer_->cancel();
    }
    trashTurtles.clear();
    teleopTurtle.reset();
    activeFollower.reset();
}
