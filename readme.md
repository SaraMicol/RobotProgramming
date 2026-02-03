### 2D Multi Robot ROS simulator

This project consists of the development of a configurable 2D multi-robot simulator written in C++ and fully integrated with ROS.
The simulator supports grid-based maps, unicycle-like mobile robots and 2D laser scanners.
All elements of the simulation are described through a human-readable configuration file.
The file specifies the simulation environment, the robots and the mounted devices, including their frame identifiers and ROS topics.
For each device, relative poses with respect to a parent frame can be defined.
Laser scanners are configurable in terms of number of beams and minimum and maximum range.
Each robot model includes limits on linear and angular velocities.
The simulator publishes standard ROS messages and TF transforms.
The final system is compatible with the ROS navigation stack and can be used for multi-robot navigation experiments.
The correct and stable code is maintained on the `master` branch.

To run the simulator, follow these steps:
```
cd ~/RobotProgramming


catkin_make


source devel/setup.bash


roslaunch ros_2d_multi_robot_simulator navigation.launch

