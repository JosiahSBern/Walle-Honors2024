#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("robot_dog_node");

    // Instantiate GameEnvironment
    auto gameEnvironment = std::make_shared<GameEnvironment>(node, "turtle1");

    // Call the method to draw the game environment
    gameEnvironment->drawGame();

    // Spin to keep the program alive for callbacks (e.g., service calls)
    rclcpp::spin(node);

    // Shutdown ROS 2 gracefully
    rclcpp::shutdown();

    return 0;
}
