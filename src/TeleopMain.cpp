int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("teleop_turtle_node");

    TeleopTurtle teleopTurtle(node, "TeleopTurtle", 0.5);
    teleopTurtle.keyLoop();

    rclcpp::shutdown();
    return 0;
}