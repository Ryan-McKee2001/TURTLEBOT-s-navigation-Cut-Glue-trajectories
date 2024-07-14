#!/bin/bash

if [ -n "$1" ]; then
    if [ "$1" = "dijkstra" ]; then
        echo "Staring $1 navigation simulation"
        ros2 launch nav2_bringup tb3_simulation_launch.py \
        headless:=False \
        map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/tb3_world/map.yaml \
        params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_dijkstra_params.yaml \
        robot_name:=turtlebot3_burger \
        robot_sdf:=$GAZEBO_MODEL_PATH/$TURTLEBOT3_MODEL/model.sdf

    elif [ "$1" = "a_star" ]; then
            echo "Starting $1 navigation simulation"
            ros2 launch nav2_bringup tb3_simulation_launch.py \
            headless:=False \
            map:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/maps/tb3_world/map.yaml \
            params_file:=$HOME/ros2_ws/src/turtlebot_navigation-cut_and_glue_trajectories/params/nav2_a_star_params.yaml \
            robot_name:=turtlebot3_burger \
            robot_sdf:=$GAZEBO_MODEL_PATH/$TURTLEBOT3_MODEL/model.sdf
    else
        echo "Parameter must be one of: dijkstra or a_star"
    fi
else
    echo "Parameter is missing"
fi