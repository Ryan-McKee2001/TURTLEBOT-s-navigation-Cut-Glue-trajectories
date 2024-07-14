import numpy as np
import random

class Node:
    """
    Node class for representing nodes in the RRT tree.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT:
    """
    RRT class for generating a path using the Rapidly-exploring Random Tree algorithm.
    """
    def __init__(self, grid, obstacle_threshold=200, max_iter=2500, step_size=2):
        """
        Initialize the RRT planner.

        Args:
            grid (list): 2D grid representing the environment where 0 indicates free space and obstacle_threshold indicates an obstacle.
            obstacle_threshold (int): Value above which a cell in the grid is considered an obstacle.
            max_iter (int): Maximum number of iterations to perform for path planning.
            step_size (int): Distance between nodes in the RRT tree.
        """
        self.grid = grid
        self.obstacle_threshold = obstacle_threshold
        self.max_iter = max_iter
        self.step_size = step_size
        self.nodes = []  # List to store all nodes in the RRT tree


    def find_path(self, start, end):
            """
            Generate a path from start to end using the RRT algorithm.

            Args:
                start (tuple): Start coordinates (x, y).
                end (tuple): End coordinates (x, y).

            Returns:
                list: Path as a list of coordinates from start to end, or None if no path was found within max_iter.
            """
            self.start = Node(start[0], start[1])
            self.end = Node(end[0], end[1])
            self.nodes = [self.start]

            for _ in range(self.max_iter):
                rnd_node = self.random_node()
                nearest_node = self.nearest_node(rnd_node)
                new_node = self.steer(nearest_node, rnd_node)

                if self.is_valid_node(new_node):
                    self.nodes.append(new_node)
                    new_node.parent = nearest_node
                    if self.is_goal_reached(new_node):
                        return self.construct_path(new_node)

            return None, None  # Path not found within max_iter

    def random_node(self):
        """
        Generate a random node in the grid.

        Returns:
            Node: Randomly generated node.
        """
        if random.random() < 0.1:  # Bias towards the goal with 10% probability
            return self.end
        elif random.random() < 0.2:  # Bias towards low weight vertices with 10% probability
            return self.get_low_weight_vertex()
        else:
            x = np.random.randint(0, len(self.grid[0]))
            y = np.random.randint(0, len(self.grid))
            return Node(x, y)

    def get_low_weight_vertex(self):
        """
        Get a vertex with low edge weight.

        Returns:
            Node: Node with low edge weight.
        """
        while True:
            x = np.random.randint(0, len(self.grid[0]))
            y = np.random.randint(0, len(self.grid))
            if self.grid[y][x] < 128:  # Assuming values less than 128 are considered low weight
                return Node(x, y)

    def nearest_node(self, rnd_node):
        """
        Find the nearest node in the RRT tree to the randomly generated node.

        Args:
            rnd_node (Node): Randomly generated node.

        Returns:
            Node: Nearest node in the RRT tree.
        """
        min_dist = float('inf')
        nearest = None
        for node in self.nodes:
            dist = np.sqrt((node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def steer(self, from_node, to_node):
        """
        Steer from from_node towards to_node with a maximum distance of step_size.

        Args:
            from_node (Node): Node to steer from.
            to_node (Node): Node to steer towards.

        Returns:
            Node: New node in the direction from from_node to to_node with a maximum distance of step_size.
        """
        dist = np.sqrt((to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2)
        if dist <= self.step_size:
            return to_node
        else:
            theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
            x = from_node.x + self.step_size * np.cos(theta)
            y = from_node.y + self.step_size * np.sin(theta)
            return Node(int(x), int(y))

    def is_valid_node(self, node):
        """
        Check if the node is valid (i.e., within the grid and not in an obstacle).

        Args:
            node (Node): Node to check.

        Returns:
            bool: True if the node is valid, False otherwise.
        """
        x, y = int(node.x), int(node.y)
        if x < 0 or x >= len(self.grid[0]) or y < 0 or y >= len(self.grid):
            return False
        return self.grid[y][x] < self.obstacle_threshold  # Check if the cell value is above or equal to the obstacle threshold

    def is_goal_reached(self, node):
        """
        Check if the goal has been reached.

        Args:
            node (Node): Node to check.

        Returns:
            bool: True if the goal has been reached, False otherwise.
        """
        return node.x == self.end.x and node.y == self.end.y

    def construct_path(self, node):
            """
            Construct the path from start to end and calculate the cost.

            Args:
                node (Node): Node to start constructing the path from (usually the goal node).
                cost_threshold (int): The maximum allowable cost for a cell in the path.

            Returns:
                tuple: A tuple containing the path as a list of tuples from start to end and the total cost.
            """
            path = [(node.x, node.y)]
            total_cost = self.grid[node.y][node.x]  # Start with the cost of the goal node
            if total_cost > self.obstacle_threshold:
                return [], 0  # Return an empty path and 0 cost if the goal node's cost is too high
            while node.parent:
                node = node.parent
                cell_cost = self.grid[node.y][node.x]
                if cell_cost > self.obstacle_threshold:
                    return path[::-1], total_cost  # Return the path up to this point and the total cost
                path.append((node.x, node.y))
                total_cost += cell_cost  # Add the cost of the current node
            return path[::-1], total_cost  # Reverse the path for start to end order