#!/bin/bash

if [ -n "$1" ]; then
    if [ "$1" = "dijkstra" ] || [ "$1" = "rrt" ] || [ "$1" = "rrt_star" ] || [ "$1" = "a_star" ]; then
        echo "Starting $1 simulation now.........."
        gnome-terminal -- bash -c "ros2 launch nav2_bringup tb3_simulation_launch.py map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/real_world/map.yaml use_sim_time:=False use_simulator:=False params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_custom_global_planner_params.yaml"
        gnome-terminal -- bash -c "ros2 run custom_planner_server planner_server --planner $1"
    else
        echo "Parameter must be one of: dijkstra, rrt, rrt_star, or a_star"
    fi
else
    echo "Parameter is missing"
fi