#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("environment_node");

    auto gameEnvironment = std::make_shared<GameEnvironment>(node, "TeleopTurtle");

    gameEnvironment->drawGame();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
