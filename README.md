# DogRobot
DogRobot is a prototype assistive robot designed to help visually impaired individuals navigate their surroundings. Using real-time mapping and voice feedback, the robot guides users to the best exit in a room. The project leverages TurtleSim in ROS2 for simulation and the Espeak library for text-to-speech functionality.

## General Objective
Assist blind individuals by allowing the robot to:




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

## Requirements
ROS2
TurtleSim
Espeak (C++ library)
C++11 or later

