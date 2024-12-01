#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include <vector>
#include <string>

class GameEnvironment : public Environment {
private:
    std::vector<Point> binPositions; // Bin positions

public:
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    void drawGame();
    void drawWalls();
    void drawBins()
};

#endif  // GAME_ENVIRONMENT_H
