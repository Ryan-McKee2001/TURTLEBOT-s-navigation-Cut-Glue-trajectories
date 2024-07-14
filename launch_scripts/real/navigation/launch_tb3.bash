#!/bin/bash

if [ -n "$1" ]; then
    if [ "$1" = "dijkstra" ]; then
        echo "Staring $1 navigation simulation"
        ros2 launch nav2_bringup tb3_simulation_launch.py map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/real_world/map.yaml use_sim_time:=False use_simulator:=False params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_dijkstra_params.yaml
    elif [ "$1" = "a_star" ]; then
            echo "Starting $1 navigation simulation"
            ros2 launch nav2_bringup tb3_simulation_launch.py map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/real_world/map.yaml use_sim_time:=False use_simulator:=False params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_a_star_params.yaml
    else
        echo "Parameter must be one of: dijkstra or a_star"
    fi
else
    echo "Parameter is missing"
fi