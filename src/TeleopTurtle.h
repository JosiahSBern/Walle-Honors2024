#ifndef TELEOP_TURTLE_H
#define TELEOP_TURTLE_H

#include "Turtle.h"
#include <memory>
#include <termios.h>

class TeleopTurtle : public Turtle {
public:
    // Constructor
    TeleopTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius);

    // Destructor
    ~TeleopTurtle();

    //Move method
    void move();

    //Render turtle status
    void renderTurtle();

    //Key event handling loop
    void keyLoop();

private:
    int kfd;
    struct termios cooked, raw;
    void handleKeyInput(char c);
    void setupTerminal();
    void restoreTerminal();
};

#endif 
