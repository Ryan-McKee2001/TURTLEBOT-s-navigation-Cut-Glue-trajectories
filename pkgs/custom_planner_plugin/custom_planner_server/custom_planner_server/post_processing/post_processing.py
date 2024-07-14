from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from copy import deepcopy
from scipy.special import comb
import time
import numpy as np

class PostProcessing():

    @staticmethod
    def create_path_message(path):
        """Create a Float32MultiArray message from a path."""
        path_msg = Float32MultiArray()
        path_msg.layout.dim.append(MultiArrayDimension(label='2D Path coordinates', size=len(path), stride=2))
        flattened_path = [float(coord) for point in path for coord in point]
        path_msg.data = flattened_path
        return path_msg
    
    # post processing function for smoothing the global path using gradient ascent: (https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
    @staticmethod
    def smooth_path(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
        """
        Creates a smooth path for a n-dimensional series of coordinates.
        Arguments:
            path: List containing coordinates of a path
            weight_data: Float, how much weight to update the data (alpha)
            weight_smooth: Float, how much weight to smooth the coordinates (beta).
            tolerance: Float, how much change per iteration is necessary to keep iterating.
        Output:
            new: List containing smoothed coordinates.
        """

        new = deepcopy(path)
        dims = len(path[0])
        change = tolerance

        while change >= tolerance:
            change = 0.0
            for i in range(1, len(new) - 1):
                for j in range(dims):

                    x_i = path[i][j]
                    y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                    y_i_saved = y_i
                    y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                    new[i][j] = y_i

                    change += abs(y_i - y_i_saved)

        return new
    
    
    @staticmethod
    def _bernstein_poly(i, n, t):
        """
        Bernstein polynomial for index i, degree n and parameter t.
        """
        return comb(n, i) * ( t**(n-i) ) * ( (1 - t)**i )

    @staticmethod
    def bezier_curve(points, n_points):
        """
        Compute points on a Bezier curve given control points.
        """
        n = len(points) - 1
        curve = np.zeros((n_points, 2))
        
        for i in range(n_points):
            t = i / (n_points - 1)
            curve[i] = np.sum([PostProcessing.bernstein_poly(j, n, t) * np.array(points[j]) for j in range(n + 1)], axis=0)
        
        return curve

    