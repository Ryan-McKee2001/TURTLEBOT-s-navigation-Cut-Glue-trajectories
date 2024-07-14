import numpy as np
import cv2

class PreProcessing():
    """Pre-processing utilities for the custom planner server."""
    @staticmethod
    def pose_stamped_to_x_y_coordinates(pose):
        """Convert a PoseStamped message to (x, y) coordinates."""
        return pose.pose.position.x, pose.pose.position.y

    @staticmethod
    def convert_occupancy_grid_to_2d_list(occupancy_grid, height, width):
        """Convert an OccupancyGrid message to a 2D list."""
        grid = np.frombuffer(occupancy_grid, dtype=np.uint8).reshape((height, width))
        return grid
    
    @staticmethod
    def cell_to_pose(cell, resolution, origin):
        """Convert a cell index to a pose."""
        x_pose = cell[0] * resolution + origin[0]
        y_pose = cell[1] * resolution + origin[1]
        return x_pose, y_pose
    
    @staticmethod
    def pose_to_cell(pose, origin, resolution):
        """Convert a pose to a cell index."""
        x_index = int((pose[0] - origin[0]) / resolution)
        y_index = int((pose[1] - origin[1]) / resolution)
        return (x_index, y_index)
    
    @staticmethod
    def threshold_occupancy_grid(grid, threshold):

        for y in range(len(grid)):
            for x in range(len(grid[0])):
                if grid[y][x] > threshold:
                    grid[y][x] = 1
                else:
                    grid[y][x] = 0

        return grid

    @staticmethod
    def dilate_occupancy_grid(grid, dilation_size, cost_threshold=254):
        """Dilate the occupancy grid."""
        # Create a binary grid where cells with value equal to cost_threshold are set to 1
        binary_grid = np.where(grid == cost_threshold, 1, 0).astype(np.uint8)

        # Create a dilation kernel
        kernel = np.ones((dilation_size, dilation_size), np.uint8)

        # Dilate the binary grid
        dilated_binary_grid = cv2.dilate(binary_grid, kernel, iterations=1)

        # Create a new grid where dilated cells have cost_threshold added to their original value
        dilated_grid = np.where(dilated_binary_grid == 1, grid + cost_threshold, grid)

        # Ensure that the values in the grid do not exceed the maximum value of 255
        dilated_grid = np.clip(dilated_grid, 0, 255)

        return dilated_grid