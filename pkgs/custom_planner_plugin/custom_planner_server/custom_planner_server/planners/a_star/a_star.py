import os
import pandas as pd
import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)

class AStar:

    def __init__(self, grid, obstacle_threshold=245):
        self.grid = grid
        self.obs_threshold = obstacle_threshold

    @staticmethod
    def manhattan_distance(start, goal):
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def find_path(self, start, goal):
        open_list = []
        closed_list = set()
        g_values = {}  # Store g values for efficient lookup

        start_node = Node(start)
        goal_node = Node(goal)

        heapq.heappush(open_list, start_node)
        g_values[start_node.position] = start_node.g

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.add(current_node)

            if current_node == goal_node:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1], None

            neighbors = [
                (current_node.position[0] + 1, current_node.position[1]),
                (current_node.position[0] - 1, current_node.position[1]),
                (current_node.position[0], current_node.position[1] + 1),
                (current_node.position[0], current_node.position[1] - 1)
            ]

            for neighbor_pos in neighbors:
                if not (0 <= neighbor_pos[0] < len(self.grid) and 0 <= neighbor_pos[1] < len(self.grid[0])):
                    continue
                if self.grid[neighbor_pos[0]][neighbor_pos[1]] >= self.obs_threshold:
                    continue

                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_g = current_node.g + self.grid[neighbor_pos[0]][neighbor_pos[1]]

                if neighbor_pos in g_values and neighbor_g >= g_values[neighbor_pos]:
                    continue  # Skip if a shorter path to this neighbor has been found

                neighbor_node.g = neighbor_g
                neighbor_node.h = self.manhattan_distance(neighbor_pos, goal)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                g_values[neighbor_pos] = neighbor_g  # Update g value

                heapq.heappush(open_list, neighbor_node)

        return None, None


if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.realpath(__file__))
    csv_file_path = os.path.join(current_dir, 'tests', 'test_grids', 'tb3_world_costmap.csv')
    grid_df = pd.read_csv(csv_file_path, header=None)
    grid = grid_df.values.tolist()
    start = (59, 26)
    goal = (96, 60)

    astar = AStar(grid=grid)
    path = astar.find_path(start, goal)
    if path:
        print("Path found:", path)
    else:
        print("No path found.")
