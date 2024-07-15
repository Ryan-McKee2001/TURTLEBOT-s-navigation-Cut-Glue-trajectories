# Commander API Node for Robot Navigation

This Python package provides a ROS 2 node that utilizes the `navigation2` framework to command a robot to navigate to multiple predefined goals. It uses `geometry_msgs` for defining poses and `nav2_simple_commander` for handling navigation tasks.

## Usage

### Prerequisites

- ROS 2 installed (Foxy or later)
- Python 3.6+
- `navigation2` and related dependencies

### Installation

1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_folder>
   '''

2. Install dependencies (if not already installed): 
  '''bash
   rosdep install -i --from-path src --rosdistro humble
   colcon build --packages-select <packages_name>
   '''

Running the Node
To run the commander API node:

1. Source your ROS 2 setup file:
  '''bash
  source /opt/ros/humble/setup.bash
  '''

2. Launch the node:
  '''bash
   ros2 run custom_planner_commander_tests commander_tests


Certainly! Here's how the README.md file would look in Markdown format:

markdown

# Commander API Node for Robot Navigation

This Python package provides a ROS 2 node that utilizes the `navigation2` framework to command a robot to navigate to multiple predefined goals. It uses `geometry_msgs` for defining poses and `nav2_simple_commander` for handling navigation tasks.

## Usage

### Prerequisites

- ROS 2 installed (Foxy or later)
- Python 3.6+
- `navigation2` and related dependencies

### Installation

1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_folder>

    Install dependencies (if not already installed):

    bash

    rosdep install -i --from-path src --rosdistro <distro_name>
    colcon build --packages-select <package_name>

Running the Node

To run the commander API node:

    Source your ROS 2 setup file:

    bash

source /opt/ros/<distro_name>/setup.bash

Launch the node:

bash

    ros2 run <package_name> <script_name>.py

Functionality

The PlannerTestingNode in the script initializes a basic navigator (BasicNavigatorWithPose) that subscribes to robot pose updates (/amcl_pose). It then sets an initial pose and navigates sequentially to each goal defined in self.goal_poses. For each goal, it waits until navigation is complete or manually canceled.
Goals Configuration

Modify the self.goal_poses dictionary in PlannerTestingNode.__init__() to specify your robot's navigation goals. Each goal should have:

    x: X-coordinate of the goal in the map frame.
    y: Y-coordinate of the goal in the map frame.
    w: Orientation (quaternion w component) of the goal.
    time: Estimated time to reach the goal (initialized to 0.0).

Notes

    Ensure proper initialization of ROS 2 environment and dependencies before running the node.
    Adjust navigation parameters and timeout values (navigation_time) as needed in the script.

For more details on navigation2 and ROS 2, refer to the official documentation and community resources.

   
  
