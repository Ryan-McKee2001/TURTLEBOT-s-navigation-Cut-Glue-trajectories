# Standard library imports
import time
import traceback
import argparse

# Third-party imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from custom_planner_interfaces.srv import CustomPlannerInterface

# Local application imports
from .planners.dijkstra.dijkstra import Dijkstra
from .planners.a_star.a_star import AStar
from .planners.rrt.rrt import RRT
from .planners.rrt_star.rrt_star import RRTStar
from .pre_processing.pre_processing import PreProcessing
from .post_processing.post_processing import PostProcessing
from .utilities.utilities import Utilities as Utils


class PlannerServer(Node):
    """PlannerServer is a ROS node that serves path planning requests."""

    def __init__(self, node_name, planner, save_log ,log_file):
        """Initialize the PlannerServer node."""
        super().__init__(node_name)
        self.planner = planner
        self.save_log = save_log
        self.log_file = log_file
        self.get_logger().info(f'{self.planner} Planner Server Node has Started........')
        self.planner_srv = self.create_service(CustomPlannerInterface, 'custom_planner_server', self._planner_callback)


    def _planner_callback(self, request, response):
        """Handles the service path request and returns the response."""
        self.get_logger().info('Received request........'); start_time = time.time()

        # Retrieve Occupany Grid Metadata and Convert to 2D List For Path Planner Algorithms
        resolution = request.occupancy_grid.info.resolution; origin = (request.occupancy_grid.info.origin.position.x, request.occupancy_grid.info.origin.position.y)
        grid = PreProcessing.convert_occupancy_grid_to_2d_list(request.occupancy_grid.data, request.occupancy_grid.info.height, request.occupancy_grid.info.width)
        # grid = PreProcessing.dilate_occupancy_grid(grid, dilation_size=2, cost_threshold=254)

        # Convert Start and Goal Poses to Grid Cell Coordinates: start_cell(x,y), goal_cell(x,y)
        start_cell = PreProcessing.pose_to_cell(pose=PreProcessing.pose_stamped_to_x_y_coordinates(pose=request.start_pose), origin=origin, resolution=resolution)
        goal_cell = PreProcessing.pose_to_cell(pose=PreProcessing.pose_stamped_to_x_y_coordinates(pose=request.end_pose), origin=origin, resolution=resolution)

        # Call the Planner Algorithm to Find the Path: path = [(x1, y1), (x2, y2), ...]
        path = self._call_planner(start_cell, goal_cell, grid)

        if path is None: # If No Path is Found, Return a Failure Response
            response.success = False; response.path = Float32MultiArray()
            self.get_logger().info(Utils.get_response_time_msg(start_time, 'Response Sent: Path Not Found........'))
            return response
        
        # Post-Processing: Smooth the Path and Convert to ROS Message
        path = PostProcessing.smooth_path([[x, y] for x, y in path])
        # path = PostProcessing.bezier_curve([[x, y] for x, y in path])

        # Format the Path Poses, optimize and Return the Response
        path_poses = [PreProcessing.cell_to_pose(cell=(path_coordinate[0], path_coordinate[1]), resolution=resolution, origin=origin) for path_coordinate in path]
        response.success = True; response.path = PostProcessing.create_path_message(path=path_poses)
        self.get_logger().info(Utils.get_response_time_msg(start_time))

        if self.save_log: # Save the Log files if the log is enabled
            Utils.create_log_file(log_file_path=self.log_file, occupancy_grid=grid, start_pose=request.start_pose, end_pose=request.end_pose, path=path_poses)

        return response


    def _call_planner(self, start_pose, goal_pose, grid):
        '''Call the planner algorithm to find the path.'''
        self.get_logger().info(f"calling {str(self.planner)} planner.....")

        # Select the Planner Algorithm Based on the Requested Planner Type
        if self.planner == 'dijkstra':
            planner_class = Dijkstra
        elif self.planner == 'a_star': 
            planner_class = AStar
        elif self.planner == 'rrt':
            planner_class = RRT
        elif self.planner == 'rrt_star':
            planner_class = RRTStar
        if planner_class is None:
            raise ValueError('Invalid planner type')
        
        planner = planner_class(grid)
        # Generate The Path Using The Selected Planner Algorithm
        path, _ = planner.find_path(start_pose, goal_pose) 
        return path


def main():
    """Main function to start the PlannerServer node."""
    parser = argparse.ArgumentParser(description='Planner Server Node')

    parser.add_argument('--planner', choices=['dijkstra', 'a_star', 'rrt', 'rrt_star'], default='dijkstra',
                        help='Specify the planner type (dijkstra, a_star, rrt, rrt_star)')
    parser.add_argument('--save_log', type=bool, default=False, help='Specify the file name for the planner log')
    parser.add_argument('--log_file', type=str, default='log/custom_planner_logs/', help='Specify the directory to save the planner logs')

    args = parser.parse_args()

    rclpy.init(args=None)
    planner_server_node = PlannerServer('custom_planner_server', args.planner, args.save_log, args.log_file)

    try:
        rclpy.spin(planner_server_node)
    except Exception as e:
        traceback.print_exc()  
    finally:
        planner_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
