#include "TeleopTurtle.h"
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <cmath>

// Constructor
TeleopTurtle::TeleopTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name, double radius)
    : Turtle(node, name, radius), kfd(0) {
    setupTerminal();
}

// Destructor
TeleopTurtle::~TeleopTurtle() {
    restoreTerminal();
}

// Handle key input
void TeleopTurtle::handleKeyInput(char c) {
    double linear = 0.0;
    double angular = 0.0;

    switch (c) {
        case 'w':  // Forward
            linear = 1.0;
            break;
        case 's':  // Backward
            linear = -1.0;
            break;
        case 'a':  // Turn left
            angular = 1.0;
            break;
        case 'd':  // Turn right
            angular = -1.0;
            break;
        case 'q':  // Quit
            restoreTerminal();
            rclcpp::shutdown();
            exit(0);
        default:
            break;
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear;
    twist_msg.angular.z = angular;
    twist_pub_->publish(twist_msg);

    // Update position for tracking purposes
    position.x += linear * std::cos(angular);
    position.y += linear * std::sin(angular);
}

// Key input loop
void TeleopTurtle::keyLoop() {
    char c;

    puts("Reading from keyboard. Use 'WASD' to move the turtle. Press 'Q' to quit.");

    while (true) {
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
        handleKeyInput(c);
    }
}

// Set up terminal for raw input
void TeleopTurtle::setupTerminal() {
    tcgetattr(kfd, &cooked);
    raw = cooked;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
}

// Restore terminal settings
void TeleopTurtle::restoreTerminal() {
    tcsetattr(kfd, TCSANOW, &cooked);
}

// Render turtle status
void TeleopTurtle::renderTurtle() {
    RCLCPP_INFO(node_->get_logger(), "TeleopTurtle: %s is at position (%f, %f)", 
                name.c_str(), position.x, position.y);
}

// Overriding move (optional additional logic could be added here)
void TeleopTurtle::move() {
    keyLoop();
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("teleop_turtle_node");

    TeleopTurtle teleopTurtle(node, "TeleopTurtle", 0.5);
    teleopTurtle.keyLoop();

    rclcpp::shutdown();
    return 0;
}

