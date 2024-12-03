#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H
#include "Environment.h"
#include "TrashTurtle.h"
#include <vector>

class GameEnvironment : public Environment {
private:
    std::vector<Point> binPositions; // Bin positions
    std::vector<std::shared_ptr<TrashTurtle>> trashTurtles; // TrashTurtle instances

public:
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    void drawGame();       // Draw the game environment
    void drawWalls();      // Draw the walls
    void drawBins();       // Draw the bins

    void spawnTrashTurtles(); // Spawn TrashTurtles
    void updateTrashTurtles(); // Update TrashTurtle positions
};
#endif  // GAME_ENVIRONMENT_H
