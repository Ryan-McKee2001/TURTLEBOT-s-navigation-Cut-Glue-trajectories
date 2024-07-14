from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# to launch: ros2 run custom_planner_server planner_server --planner <planner_type>
def generate_launch_description():
    launch_dir = get_package_share_directory('custom_planner_server')

    declare_planner = DeclareLaunchArgument(
        '--planner',
        default_value='dijkstra',
        description='Planner to use: rrt, rrt_star, a_star, or dijkstra'
    )

    planner = LaunchConfiguration('--planner')

    # Create the Node action
    planner_node = Node(
        package='custom_planner_server',
        executable='planner_server',
        output='screen',
        arguments=['--planner', planner],
    )

    start_planner_server_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'custom_planner_server', 'planner_server', '--planner:=', planner],
        cwd=[launch_dir], output='screen')

    return LaunchDescription([
        declare_planner,
        planner_node,
        start_planner_server_cmd
    ])