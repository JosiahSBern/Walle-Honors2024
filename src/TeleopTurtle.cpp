#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71

class TeleopTurtle
{
public:
  TeleopTurtle(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:
  std::shared_ptr<rclcpp::Node> nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

TeleopTurtle::TeleopTurtle(std::shared_ptr<rclcpp::Node> nh)
    : nh_(nh),
      linear_(0),
      angular_(0),
      l_scale_(2.0),
      a_scale_(2.0)
{
  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_turtle_wasd");
  TeleopTurtle teleop_turtle(node);

  signal(SIGINT, quit);

  teleop_turtle.keyLoop();

  return 0;
}

void TeleopTurtle::keyLoop() {
    char c;
    bool dirty = false;

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Use WASD keys to move the turtle. Q to quit.");

    for (;;) {
        if (::read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        linear_ = angular_ = 0;

        switch (c) {
            case KEYCODE_W: linear_ = 1.0; dirty = true; break;
            case KEYCODE_S: linear_ = -1.0; dirty = true; break;
            case KEYCODE_A: angular_ = 1.0; dirty = true; break;
            case KEYCODE_D: angular_ = -1.0; dirty = true; break;
            case KEYCODE_Q: quit(SIGINT); break;
        }

        geometry_msgs::msg::Twist twist;
        twist.angular.z = a_scale_ * angular_;
        twist.linear.x = l_scale_ * linear_;
        if (dirty) {
            twist_pub_->publish(twist);

            // Simulate position updates
            position.x += linear_ * std::cos(angular_);
            position.y += linear_ * std::sin(angular_);

            dirty = false;
        }
    }
}

  char c;
  bool dirty = false;

  // Get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use WASD keys to move the turtle. Q to quit.");

  for (;;)
  {
    // Get the next event from the keyboard
    if (::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_ = angular_ = 0;

    switch (c)
    {
    case KEYCODE_W:
      std::cout << "FORWARD" << std::endl;
      linear_ = 1.0;
      dirty = true;
      break;
    case KEYCODE_S:
      std::cout << "BACKWARD" << std::endl;
      linear_ = -1.0;
      dirty = true;
      break;
    case KEYCODE_A:
      std::cout << "LEFT" << std::endl;
      angular_ = 1.0;
      dirty = true;
      break;
    case KEYCODE_D:
      std::cout << "RIGHT" << std::endl;
      angular_ = -1.0;
      dirty = true;
      break;
    case KEYCODE_Q:
      std::cout << "QUIT" << std::endl;
      quit(SIGINT);
      break;
    }

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_ * angular_;
    twist.linear.x = l_scale_ * linear_;
    if (dirty)
    {
      twist_pub_->publish(twist);
      dirty = false;
    }
  }
}
