#!/bin/bash

echo "Starting SLAM....."
gnome-terminal -- bash -c 'ros2 launch turtlebot3_cartographer cartographer.launch.py'
gnome-terminal -- bash -c 'ros2 run turtlebot3_teleop teleop_keyboard'