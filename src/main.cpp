#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Environment Builder...");

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("environment_builder");

    // Define a turtle name
    std::string turtle_name = "turtle1";

    // Create a GameEnvironment instance
    auto environment = std::make_shared<GameEnvironment>(node, turtle_name);

    // Draw the game environment
    environment->drawGame();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Game environment setup complete.");

    // Spin the node to keep it active and process events
    rclcpp::spin(node);

    // Shut down ROS 2
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down Environment Builder...");
    rclcpp::shutdown();

    return 0;
}