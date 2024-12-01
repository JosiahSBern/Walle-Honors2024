#include "UserTurtle.h"
#include "CollisionHandler.h"
#include "Point.h"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71

UserTurtle::UserTurtle(std::shared_ptr<rclcpp::Node> node)
    : nh_(node), linear_(0), angular_(0), l_scale_(2.0), a_scale_(2.0) {
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
}

void UserTurtle::keyLoop() {
    char c;
    struct termios cooked, raw;

    // Set the console in raw mode
    tcgetattr(STDIN_FILENO, &cooked);
    raw = cooked;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    CollisionHandler collisionHandler(11.0, 11.0);  // Room boundaries
    Point currentPos = {5.5, 5.5};

    puts("Use WASD keys to move the turtle. Press Q to quit.");

    while (true) {
        if (read(STDIN_FILENO, &c, 1) < 0) break;
        if (c == KEYCODE_Q) break;

        Point proposedPos = currentPos;

        switch (c) {
            case KEYCODE_W: proposedPos.y += 0.5; break;
            case KEYCODE_S: proposedPos.y -= 0.5; break;
            case KEYCODE_A: proposedPos.x -= 0.5; break;
            case KEYCODE_D: proposedPos.x += 0.5; break;
        }

        currentPos = collisionHandler.adjustPositionWithSoftCollision(currentPos, proposedPos);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = (proposedPos.x != currentPos.x) ? 0 : l_scale_ * linear_;
        twist.angular.z = (proposedPos.y != currentPos.y) ? 0 : a_scale_ * angular_;
        twist_pub_->publish(twist);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
}
