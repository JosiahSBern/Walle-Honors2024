#include "GameEnvironment.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    //Initialize ROS 2 
    rclcpp::init(argc, argv);

    //create main node
    auto node = std::make_shared<rclcpp::Node>("trash_sorting_game_node");

    auto gameEnvironment = std::make_shared<GameEnvironment>(node, "turtle1");

    gameEnvironment->drawGame();

    rclcpp::spin(node);

    //Shutdown ROS 2 communication
    rclcpp::shutdown();

    return 0;
}
