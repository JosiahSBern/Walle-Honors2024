#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include "TrashTurtle.h"
#include "Point.h"
#include <vector>
#include <memory>

class GameEnvironment : public Environment {
private:
    std::vector<Point> binPositions;  // Positions of bins
    std::vector<std::shared_ptr<TrashTurtle>> trashTurtles;  // TrashTurtle instances
    std::shared_ptr<Turtle> turtle1;  // Leader turtle

    void spawnTrashTurtles();
    void updateTrashTurtles();

public:
    GameEnvironment(rclcpp::Node::SharedPtr node, const std::string& turtle_name);
    void drawGame() override;
    void drawWalls() override;
    void drawBins();
};

#endif  // GAME_ENVIRONMENT_H
