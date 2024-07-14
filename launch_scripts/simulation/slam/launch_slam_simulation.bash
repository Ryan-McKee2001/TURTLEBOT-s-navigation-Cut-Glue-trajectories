#!/bin/bash

echo "Starting SLAM simulation"
export TURTLEBOT3_MODEL=burger

gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
gnome-terminal -- bash -c "ros2 run turtlebot3_teleop teleop_keyboard"
gnome-terminal -- bash -c "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"