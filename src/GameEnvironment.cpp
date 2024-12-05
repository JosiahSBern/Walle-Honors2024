#include <rclcpp/rclcpp.hpp>
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include <memory>
#include "GameEnvironment.h"
#include "TeleopTurtle.h"
#include "TrashTurtle.h"
#include "TrashType.h"
#include <cmath>
#include <limits>

GameEnvironment::GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name)
    : Environment(node, turtle_name) {
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
}

GameEnvironment::~GameEnvironment() {
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    trashTurtles.clear();
    teleopTurtle.reset();
    activeFollower.reset();
}

void GameEnvironment::drawGame() {
    RCLCPP_INFO(node_->get_logger(), "Drawing game...");

    // Step 1: Draw static elements
    drawWalls();
    drawBins();

    // Step 2: Spawn initial TrashTurtles
    spawnTrashTurtles();

    // Step 3: Remove `turtle1` and spawn a new turtle in the center
    initializeEnvironment();

    // Start the update timer
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() { updateTrashTurtles(); }
    );
}

void GameEnvironment::initializeEnvironment() {
    // Kill the default turtle1
    if (!kill_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Kill service not available.");
    } else {
        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = "turtle1";

        auto kill_result = kill_client_->async_send_request(kill_request);
        if (rclcpp::spin_until_future_complete(node_, kill_result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to kill turtle1.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully removed turtle1.");
        }
    }

    // Spawn a new TeleopTurtle in the center
    if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Spawn service not available.");
    } else {
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = 5.5;
        spawn_request->y = 5.5;
        spawn_request->theta = 0.0;
        spawn_request->name = "teleop_turtle";

        auto spawn_result = spawn_client_->async_send_request(spawn_request);
        if (rclcpp::spin_until_future_complete(node_, spawn_result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to spawn teleop_turtle.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully spawned teleop_turtle at (5.5, 5.5).");

            // Instantiate the TeleopTurtle object
            teleopTurtle = std::make_shared<TeleopTurtle>(node_, "teleop_turtle", 0.5);
            teleopTurtle->setPosition({5.5, 5.5});
        }
    }
}

void GameEnvironment::spawnTrashTurtles() {
    RCLCPP_INFO(node_->get_logger(), "Spawning TrashTurtles...");

    const double bottomBoxLeft = 1.0;
    const double bottomBoxRight = 10.0;
    const double bottomBoxTop = 3.0;
    const double bottomBoxBottom = 1.0;

    const int numTurtles = 9;
    const double horizontalSpacing = (bottomBoxRight - bottomBoxLeft) / (numTurtles + 1);

    for (size_t i = 0; i < numTurtles; ++i) {
        TrashType type = static_cast<TrashType>(i % 3);  // Cycle through trash types
        std::string name = "Trash" + std::to_string(i + 1);

        double xPos = bottomBoxLeft + (i + 1) * horizontalSpacing;
        double yPos = (bottomBoxTop + bottomBoxBottom) / 2;

        // Use the /spawn service
        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = xPos;
        spawn_request->y = yPos;
        spawn_request->theta = 0.0;  // Facing straight
        spawn_request->name = name;

        if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Spawn service not available.");
            return;
        }

        auto spawn_result = spawn_client_->async_send_request(spawn_request);
        if (rclcpp::spin_until_future_complete(node_, spawn_result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to spawn TrashTurtle: %s", name.c_str());
            continue;
        }

        // Create TrashTurtle instance and add it to the list
        auto trashTurtle = std::make_shared<TrashTurtle>(
            node_,
            name,
            0.5,
            type,
            binPositions[i % binPositions.size()]
        );

        trashTurtle->setPosition({xPos, yPos});
        trashTurtles.push_back(trashTurtle);

        // Log the spawned turtle
        RCLCPP_INFO(node_->get_logger(), "Spawned TrashTurtle: %s at (%f, %f)", name.c_str(), xPos, yPos);
    }
}



void GameEnvironment::updateTrashTurtles() {
    assignFollower();  // Reassign a follower if necessary
    for (auto& turtle : trashTurtles) {
        if (turtle == activeFollower) {
            turtle->followLeader();
        } else {
            turtle->move();  // Non-followers move towards their bins
        }
    }
}

void GameEnvironment::resetFollowerPen() {
    if (activeFollower) {
        // Reset pen color of the previous follower to default (e.g., black)
        activeFollower->setPenColor(0, 0, 0, 2);
    }
}

void GameEnvironment::assignFollower() {
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
        resetFollowerPen();  // Reset previous follower's pen color

        activeFollower = closestTurtle;
        activeFollower->setLeaderTurtle(teleopTurtle);

        // Set new follower's pen color to visually distinguish it
        activeFollower->setPenColor(255, 0, 0, 3);  // Red pen with width 3

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
    drawRectangle(topLeft, bottomRight, 255, 255, 255); // White

    RCLCPP_INFO(node_->get_logger(), "Walls drawn: TopLeft (%f, %f), BottomRight (%f, %f)",
                topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
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
}


