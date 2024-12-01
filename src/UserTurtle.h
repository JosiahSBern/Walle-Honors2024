//Using teleop_turtle_key.cpp https://github.com/sukha-cn/turtlesim-ros2/blob/master/tutorials/teleop_turtle_key.cpp
#ifndef USERTURTLE_H
#define USERTURTLE_H

#include "Turtle.h"
#include <termios.h>
#include <unistd.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71

class UserTurtle : public Turtle {
private:
    double linear_, angular_, l_scale_, a_scale_;

    void setConsoleModeRaw();
    void resetConsoleMode();
    void updatePosition(char key);

public:
    UserTurtle(std::shared_ptr<rclcpp::Node> node);
    void move() override;
    void renderTurtle() override;
    ~UserTurtle() override = default;
};

#endif
