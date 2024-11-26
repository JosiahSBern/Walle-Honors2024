//Using teleop_turtle_key.cpp https://github.com/sukha-cn/turtlesim-ros2/blob/master/tutorials/teleop_turtle_key.cpp
#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#include <memory.h>
#include "rclcpp/rclcpp.hpp"


class UserTurtle
{
    public: 
        UserTurtle();
        void keyLoop();
        void quit();

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        double linear_, angular_, l_scale_, a_scale_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;


};
#endif