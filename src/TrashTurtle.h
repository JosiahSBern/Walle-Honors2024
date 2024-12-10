// include/TrashTurtle.h
#ifndef TRASH_TURTLE_H
#define TRASH_TURTLE_H

#include "Turtle.h"
#include "Point.h"
#include "TrashType.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/msg/pose.hpp"

enum class SortState {
        MOVING_TO_BIN,
        SORTING,
        SORTED
};

class TrashTurtle : public Turtle {
private:
    //Trash-specific attributes
    TrashType type;
    Point targetPosition;
    Point binPosition;
    double orientation;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;

    //Navigation parameters
    double targetRadius;
    

    //ROS2 Client for setting pen
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;


    //Helper methods
    void updateVelocityToTarget(const Point& target);
    void stopMovement();
    Point getBinPositionForTrashType() const;

    // State management
    SortState currentState_;


public:
    TrashTurtle(std::shared_ptr<rclcpp::Node> node, const std::string& name,
                double radius, TrashType type, const Point& target);

    //Core movement and navigation methods
    void move(const Turtle& target,double follow_distance);

    // Setter and getter methods
    void setTargetPosition(const Point& target);
    void setPenColor(int r, int g, int b, int width);

    TrashType getTrashType() const;
    SortState getCurrentState() const;

    //State checking methods
    bool isAtTarget() const;
    // void sortIntoBin();
};

#endif // TRASH_TURTLE_H
