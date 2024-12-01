#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include <vector>


using namespace std;
// Enum to represent object types
enum ObjectType {
    RECYCLING,
    PAPER,
    TRASH
};

// Struct for game objects
struct GameObject {
    Point position;  // Current position
    ObjectType type; // Type of the object
    string name; // Object name (e.g., "Bottle", "Can")
};

class GameEnvironment : public Environment {
private:
    std::vector<Point> binPositions;  // Positions of bins (top row)

    void drawBins();           // Draw the three color-coded bins
    
public:
    GameEnvironment(rclcpp::Node::SharedPtr node);
    void drawGame();           // Draw the game layout
    void drawWalls(); // Override drawWalls from Environment
};

#endif  // GAME_ENVIRONMENT_H
