#!/bin/bash
# Run this script to compile the 2D multi-robot simulator

# Compile the code
catkin_make

# Source the setup file to add the new package to the ROS environment
source devel/setup.bash