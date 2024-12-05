#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include "TeleopTurtle.h"
#include "TrashTurtle.h"
#include "Point.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "TrashTurtle.h"
#include "turtlesim/srv/kill.hpp"


class GameEnvironment : public Environment {
private:
    // Position of bins in the environment
    std::vector<Point> binPositions;

    // TrashTurtle instances
    std::vector<std::shared_ptr<TrashTurtle>> trashTurtles;

    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr timer_;

    // TeleopTurtle instance
    std::shared_ptr<TeleopTurtle> teleopTurtle;

    // Active follower TrashTurtle
    std::shared_ptr<TrashTurtle> activeFollower;

    // Synchronization
    std::mutex updateMutex;

    // Service clients for spawning and killing turtles
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;

    // Private methods
       // Update the state of all TrashTurtles

public:
    // Constructor
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);

    // Destructor
    ~GameEnvironment();

    // Public method to draw the environment
    void drawGame();
    void initializeEnvironment();  // Remove turtle1 and spawn a new TeleopTurtle
    void spawnTrashTurtles();      // Spawn TrashTurtles in designated areas
    void assignFollower();         // Assign the closest TrashTurtle to follow the TeleopTurtle
    void updateTrashTurtles();  
    void resetFollowerPen();
    void drawBins();
    void drawWalls();
};

#endif // GAME_ENVIRONMENT_H
