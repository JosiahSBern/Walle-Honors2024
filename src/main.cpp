#include <rclcpp/rclcpp.hpp>
#include "GameEnvironment.h"

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto node = rclcpp::Node::make_shared("environment_builder");

    // Create the game environment
    auto game_environment = std::make_shared<GameEnvironment>(node);

    // Draw the game environment
    RCLCPP_INFO(node->get_logger(), "Drawing game environment...");
    game_environment->drawGame();

    // Spin the node to keep it alive and allow visualization
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}