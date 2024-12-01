#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("environment_builder");

    std::string turtle_name = "turtle1";  // Replace with your turtle's name
    auto environment = std::make_shared<GameEnvironment>(node, turtle_name);

    environment->drawGame();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
