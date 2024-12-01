#ifndef GAME_ENVIRONMENT_H
#define GAME_ENVIRONMENT_H

#include "Environment.h"
#include <vector>
#include <string>

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
    std::string name; // Object name (e.g., "Bottle", "Can")
};

class GameEnvironment : public Environment {
private:
    std::vector<GameObject> objects;   // List of objects in the assortment box
    std::vector<Point> binPositions;  // Positions of bins (top row)

    void drawBins();           // Draw the three color-coded bins
    void populateAssortment(); // Populate objects in the assortment box
    void handleSorting();      // Logic for sorting objects into bins

public:
    GameEnvironment(rclcpp::Node::SharedPtr node);
    void drawGame();           // Draw the game layout
};

#endif
