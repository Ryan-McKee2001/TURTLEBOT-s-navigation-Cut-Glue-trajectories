import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
          Node(
            package='wall_follower',
            executable='wall_follower',
            output='screen'
          )
    ])
    