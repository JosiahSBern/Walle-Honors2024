#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include <vector>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "Environment.h"
#include "Point.h"
#include "TeleopTurtle.h"
#include "TrashTurtle.h"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

class GameEnvironment : public Environment {
private:
    // Positions of bins in the environment
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

public:
    // Constructor
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);

    // Destructor
    ~GameEnvironment();

    // Public methods
    void drawGame();
    // void initializeEnvironment(); // Remove turtle1 and spawn a new TeleopTurtle
    void spawnTrashTurtles();     // Spawn TrashTurtles in designated areas
    void assignFollower();        // Assign the closest TrashTurtle to follow the TeleopTurtle
    void updateTrashTurtles();  
    void resetFollowerPen();
    void drawBins();
    void drawWalls();
};

#endif // GAME_ENVIRONMENT_H
