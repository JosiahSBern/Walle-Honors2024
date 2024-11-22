# DogRobot
DogRobot is a prototype assistive robot designed to help visually impaired individuals navigate their surroundings. Using real-time mapping and voice feedback, the robot guides users to the best exit in a room. The project leverages TurtleSim in ROS2 for simulation and the Espeak library for text-to-speech functionality.

## General Objective
Assist blind individuals by allowing the robot to:

Map out an environment.
Identify the best route to an exit.
Provide step-by-step navigation instructions via audio feedback.
Features
Environment Mapping:
The robot maps the environment, simulating real-world obstacles and exits.
Optimal Pathfinding:
Using TurtleSim, the robot calculates the shortest and safest route to a designated exit.
Text-to-Speech Feedback:
Navigation commands are read aloud to the user using the Espeak C++ library.
Interactive User Control:
Users can manually control the robot's movement using simple keyboard commands.
Prototype Details
Environment Setup
A Turtle object draws the environment, simulating walls and obstacles.
The environment includes an exit, with the robot providing a brief description, such as:
"The room has an exit to the north."
Robot’s Send Commands
The robot calculates the optimal path to the exit from its current position.
An array of commands (left, right, forward, backward) is generated based on the optimal path.
Commands are read aloud using the Espeak library, guiding the user step-by-step.
User Commands
Users can manually control a second Turtle using keyboard inputs:
W: Move forward
S: Move backward
A: Turn left
D: Turn right
The robot displays the user's current orientation:
North (forward)
South (backward)
West (left)
East (right)
If the user moves within one step of a wall, an error message is triggered and read aloud:
"Warning: Wall ahead!"
If the user deviates from the optimal path, the system recalculates a new route and updates the instructions.
Requirements
ROS2
TurtleSim
Espeak (C++ library)
C++17 or later
Installation
Set up ROS2:
Follow the official installation guide for ROS2 here.

Install TurtleSim:

bash
Copy code
sudo apt install ros-<ros2-distro>-turtlesim
Install Espeak:

bash
Copy code
sudo apt install espeak
Clone this repository:

bash
Copy code
git clone https://github.com/<your-username>/DogRobot.git
cd DogRobot
Build the package:

bash
Copy code
colcon build
Run the simulation:

bash
Copy code
source install/setup.bash
ros2 launch dog_robot_simulation launch_file_name.launch.py
Usage
Launch the environment and allow the robot to map out the space.
The robot will calculate the optimal path to the exit and read out the navigation instructions.
Use keyboard controls to move the user-controlled Turtle and navigate the environment.
Follow the robot's guidance to reach the exit safely.
Future Enhancements
Add dynamic obstacle detection and avoidance.
Implement multi-room navigation with multiple exits.
Simulate real-world sensors like LiDAR or cameras for environment mapping.
Connect the system to a mobile app for additional feedback.
