#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <filesystem>

void launchTurtlesimWithBackground(int r, int g, int b) {
    // Check if gnome-terminal is available
    if (!std::filesystem::exists("/usr/bin/gnome-terminal")) {
        RCLCPP_ERROR(rclcpp::get_logger("TurtlesimLauncher"), "gnome-terminal not found. Install it or use a different terminal.");
        return;
    }

    std::string command = "gnome-terminal -- bash -c 'ros2 run turtlesim turtlesim_node ";
    command += "--ros-args -p background_r:=" + std::to_string(r);
    command += " -p background_g:=" + std::to_string(g);
    command += " -p background_b:=" + std::to_string(b);
    command += "; exec bash'";

    int ret = std::system(command.c_str());
    if (ret == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("TurtlesimLauncher"), "Failed to launch turtlesim_node.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("TurtlesimLauncher"), "Turtlesim launched with background color (%d, %d, %d).", r, g, b);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Parse background color arguments
    int r = 30, g = 30, b = 30; // Default
    if (argc == 4) {
        r = std::stoi(argv[1]);
        g = std::stoi(argv[2]);
        b = std::stoi(argv[3]);
    }

    // Launch turtlesim with the given background color
    launchTurtlesimWithBackground(r, g, b);

    // Create the game environment node
    auto node = std::make_shared<rclcpp::Node>("trash_sorting_game_node");
    auto gameEnvironment = std::make_shared<GameEnvironment>(node, "turtle1");

    // Draw the game and start spinning
    gameEnvironment->drawGame();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
