#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a node and instantiate the GameEnvironment
    auto node = rclcpp::Node::make_shared("environment_builder");
    auto GameEnvironment = std::make_shared<GameEnvironment>(node, "turtle1");

    // Call the method to draw the game environment
    GameEnvironment->drawGame();

    // Spin to keep the program alive for callbacks (e.g., service calls)
    rclcpp::spin(node);

    // Shutdown ROS 2 gracefully
    rclcpp::shutdown();

    return 0;
}
