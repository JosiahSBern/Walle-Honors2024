#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("environment_builder");

    // Instantiate GameEnvironment object
    auto game_environment = std::make_shared<GameEnvironment>(node, "turtle1");

    // Draw the environment
    game_environment->drawGame();

    // Spin to keep the node alive
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
