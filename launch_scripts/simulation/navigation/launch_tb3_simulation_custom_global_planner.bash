#!/bin/bash

if [ -n "$1" ]; then
    if [ "$1" = "dijkstra" ] || [ "$1" = "rrt" ] || [ "$1" = "rrt_star" ] || [ "$1" = "a_star" ]; then
        echo "Starting $1 simulation now.........."
        gnome-terminal -- bash -c "ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/tb3_world/map.yaml params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_custom_global_planner_params.yaml robot_name:=turtlebot3_burger robot_sdf:=$GAZEBO_MODEL_PATH/$TURTLEBOT3_MODEL/model.sdf"
        gnome-terminal -- bash -c "ros2 run custom_planner_server planner_server --planner $1"
    else
        echo "Parameter must be one of: dijkstra, rrt, rrt_star, or a_star"
    fi
else
    echo "Parameter is missing"
fi