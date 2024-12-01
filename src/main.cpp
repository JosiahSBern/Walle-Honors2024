#include "GameEnvironment.h"
#include "UserTurtle.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("game_node");
    GameEnvironment gameEnv(node);
    UserTurtle userTurtle(node);

    gameEnv.drawGame();
    userTurtle.keyLoop();

    rclcpp::shutdown();
    return 0;
}
