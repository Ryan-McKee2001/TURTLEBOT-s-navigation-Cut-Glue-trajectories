import os
import unittest
import time
import pandas as pd
import matplotlib.pyplot as plt
from parameterized import parameterized
from ..dijkstra import Dijkstra

class TestDijkstraIntegration(unittest.TestCase):
    pos_test_start_coords = [(51, 21), (20, 50), (75, 75), (96, 75), (75, 96)]
    pos_test_end_coords = [(96, 75), (75, 96), (96, 75), (51, 50), (20, 50)]
    neg_test_start_coords = [(51, 50), (20, 20), (96, 96)]
    neg_test_end_coords = [(96, 96), (51, 50), (20, 20)]
    visualise_paths_arg = True

    @classmethod
    def setUpClass(cls):
        grid_df = pd.read_csv(os.getcwd() + '\\dijkstra\\tests\\test_grids\\tb3_world_costmap.csv', header=None)
        grid = grid_df.values.tolist()
        cls.planner = Dijkstra(grid)

    def setUp(self):
        self.start_coords = None
        self.end_coords = None

    def visualize_path(self, path):
        if self.visualise_paths_arg:
            # Define custom colormap
            cmap = plt.cm.viridis  # Use the Viridis colormap for a wider range of colors
            bounds = [0, 128, 192, 224, 240, 254, 255]
            norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

            # Plot grid
            plt.imshow(self.planner.grid, cmap=cmap, norm=norm, interpolation='nearest')
            if path is not None:
                path_x = [point[0] for point in path]
                path_y = [point[1] for point in path]
                plt.plot(path_y, path_x, 'r-')
            plt.plot(self.start_coords[1], self.start_coords[0], 'go')
            plt.plot(self.end_coords[1], self.end_coords[0], 'ro')
            plt.colorbar(label='Cell Value')
            plt.title('Grid Visualization')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()

    @parameterized.expand(zip(pos_test_start_coords, pos_test_end_coords))
    def test_dijkstra_shortest_path(self, start_coords, end_coords):
        self.start_coords = start_coords
        self.end_coords = end_coords
        print (f"Testing path from {start_coords} to {end_coords} on TB3 world costmap")
        start_time = time.time()
        path, cost = self.planner.find_path(start_coords, end_coords)
        end_time = time.time()
        print (f"Path: {path}")
        print (f"Cost: {cost}")
        print(f"Time taken: {end_time - start_time} seconds")
        self.visualize_path(path)
        self.assertEqual(path[-1], end_coords)

    def test_dijkstra_no_path(self):
        self.start_coords = (1, 1)
        self.end_coords = (96, 75)
        print (f"Negative Testing impossible path from {self.start_coords} to {self.end_coords} on TB3 world costmap")
        path, _ = self.planner.find_path(self.start_coords, self.end_coords)
        self.assertIsNone(path)

    def test_dijkstra_with_exception_inputs(self):
        self.start_coords = (1, -10)
        self.end_coords = (200, 75)
        print (f"Negative Testing invalid path from {self.start_coords} to {self.end_coords} on TB3 world costmap")
        path, _ = self.planner.find_path(self.start_coords, self.end_coords)
        self.assertIsNone(path)

if __name__ == '__main__':
    unittest.main()