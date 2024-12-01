#include <rclcpp/rclcpp.hpp>
#include "GameEnvironment.h"
#include <iostream>


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("environment_builder");

    // Example with dynamic turtle name
    std::string turtle_name = "turtle1";  // Replace with dynamic name if needed
    auto environment = std::make_shared<GameEnvironment>(node, turtle_name);

    environment->drawGame();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
