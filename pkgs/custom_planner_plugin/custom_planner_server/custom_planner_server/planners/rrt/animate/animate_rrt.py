import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ..rrt import RRT, Node

class RRTAnimator(RRT):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fig, self.ax = plt.subplots()

    def init_animation(self):
        self.ax.set_xlim(0, len(self.grid[0]))
        self.ax.set_ylim(0, len(self.grid))
        self.ax.imshow(self.grid, cmap='Greys', origin='lower')
        self.start_point, = self.ax.plot([], [], 'go')
        self.end_point, = self.ax.plot([], [], 'ro')
        self.tree, = self.ax.plot([], [], 'b')

    def update_animation(self, i):
        if i < len(self.nodes):
            node = self.nodes[i]
            if node.parent:
                self.tree.set_xdata(list(self.tree.get_xdata()) + [node.parent.x, node.x])
                self.tree.set_ydata(list(self.tree.get_ydata()) + [node.parent.y, node.y])
            if i == 0:
                self.start_point.set_data(node.x, node.y)
            elif i == len(self.nodes) - 1:
                self.end_point.set_data(node.x, node.y)
        return self.start_point, self.end_point, self.tree

    def animate(self):
        self.init_animation()
        ani = animation.FuncAnimation(self.fig, self.update_animation, frames=len(self.nodes)+10, interval=100, blit=True)
        plt.show()

if __name__ == "__main__":
    start = (0, 0)
    goal = (9, 20)
    grid = [[0 for _ in range(100)] for _ in range(100)]  # replace with your actual grid

    rrt_animator = RRTAnimator(grid, step_size=1)
    rrt_animator.find_path(start, goal)
    rrt_animator.animate()