#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include "TrashTurtle.h"
#include "Point.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "turtlesim/srv/kill.hpp"

class GameEnvironment : public Environment {
private:
    //Position of bins in the environment
    std::vector<Point> binPositions;

    //TrashTurtle instances
    std::vector<std::shared_ptr<TrashTurtle>> trashTurtles;

    //Timer for periodic updates
    rclcpp::TimerBase::SharedPtr timer_;

    //Service clients
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;

    //Central turtle
    std::shared_ptr<Turtle> centralTurtle;
public:
    // Constructor
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& central_turtle_name);

    // Destructor
    ~GameEnvironment();

    void drawGame();
    void updateTrashTurtles();  
    void drawBins();
    void drawWalls();
    void spawnTrashTurtles();
    void initializeEnvironment(); 
    void updateCentralTurtlePosition();
    void handleSorting();
    void moveTurtleToBins();
};

#endif 
