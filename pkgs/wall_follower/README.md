# TurtleBot3 Wall Follower Algorithm

This package contains the implementation of the `wall_follower` node algorithm. This simple algorithm utilizes Turtlebot3's laserscan data to autonomously navigate and avoid obstacles in simple environments.

## Algorithm Steps

1. **Import Necessary Libraries**: Start by importing the required libraries for ROS2, handling messages, and basic Python functionalities.

2. **Define TurnDirection Enum**: Create an enumeration to specify the robot's turning direction - clockwise, anti-clockwise, or straight.

3. **Define RobotControl Class**: Create a class to control the robot's movement by publishing velocity commands to the robot.

4. **Define WallFollower Class**: Implement a class to make the robot follow walls. This class subscribes to laser scan data, processes it, and controls the robot's movement accordingly using the RobotControl class.

5. **Define main Function**: Write a function to initialize the ROS node for wall-following behavior, handle exceptions, and start the ROS event loop.

6. **Run main Function**: If the script is run directly, execute the main function to begin the wall-following behavior.

## How to Use

To launch the demo and the node, use the following commands:

```bash
# Command for launching demo
ros2 run wall_follower wall_follower

```

# Videos:

<table>
  <tr>
    <td>
      <strong>Real World</strong><br>
      <img src="./readme_resources/wall%20follower%20real%20gif.gif" alt="Watch the Real World Video">
    </td>
    <td>
      <strong>Simulation</strong><br>
      <img src="./readme_resources/wall%20follower%20simulation%20gif.gif" alt="Simulation GIF">
    </td>
  </tr>
</table>
