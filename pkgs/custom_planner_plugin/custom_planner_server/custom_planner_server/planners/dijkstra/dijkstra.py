import heapq

class Dijkstra:
    def __init__(self, grid, obstacle_threshold=230):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.obstacle_threshold = obstacle_threshold

    def find_path(self, start, end):
        heap = [(0, start)]
        distances = {start: 0}
        paths = {start: []}
        while heap:
            (cost, current) = heapq.heappop(heap)
            if current == end:
                return paths[end], distances[end]  # Return the path and the cost to reach the end node
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                next = (current[0] + dx, current[1] + dy)
                if (0 <= next[0] < self.rows and 0 <= next[1] < self.cols):
                    if self.grid[next[0]][next[1]] >= self.obstacle_threshold:
                        continue
                    next_cost = cost + self.grid[next[0]][next[1]]
                    if next not in distances or next_cost < distances[next]:
                        distances[next] = next_cost
                        heapq.heappush(heap, (next_cost, next))
                        paths[next] = paths[current] + [next]
        return None, None
