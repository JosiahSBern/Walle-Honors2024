#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("environment_builder");

    // Define a turtle name
    std::string turtle_name = "turtle1";

    // Define a target point for the TrashTurtle
    Point someTargetPoint = {5.0, 5.0};

    // Create a TrashTurtle instance (example usage)
    auto myTurtle = std::make_shared<TrashTurtle>(node, "MyTrash", 0.5, TrashType::PLASTIC, someTargetPoint);

    // Create a GameEnvironment instance
    auto environment = std::make_shared<GameEnvironment>(node, turtle_name);

    // Draw the game environment
    environment->drawGame();

    // Spin the node
    rclcpp::spin(node);

    // Shut down ROS 2
    rclcpp::shutdown();

    return 0;
}
