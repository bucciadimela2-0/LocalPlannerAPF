import matplotlib.animation as animation
import matplotlib.pyplot as plt
from shapely.geometry import Point


class HybridPlannerAnimation:
    def __init__(self, obstacles, start_point, goal_point):
        self.obstacles = obstacles
        self.start_point = start_point
        self.goal_point = goal_point
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.frames = []
        self.setup_plot()
    
    def setup_plot(self):
        # Plot obstacles
        for obstacle in self.obstacles:
            self.ax.fill(*zip(*obstacle.exterior.coords), color='gray', alpha=0.3)
        
        # Plot start and goal points
        self.ax.plot(self.start_point.x, self.start_point.y, 'go', markersize=10, label='Start')
        self.ax.plot(self.goal_point.x, self.goal_point.y, 'ro', markersize=10, label='Goal')
        
        self.ax.grid(True)
        self.ax.set_title('Hybrid Path Planning Animation')
    
    def add_rrt_frame(self, tree, current_node=None):
        frame = []
        # Plot tree edges
        for node in tree:
            if node.parent is not None:
                line, = self.ax.plot([node.x, node.parent.x], [node.y, node.parent.y],
                                   'b-', alpha=0.3, linewidth=1)
                frame.append(line)
        
        # Highlight current node if provided
        if current_node is not None and current_node.parent is not None:
            line, = self.ax.plot([current_node.x, current_node.parent.x],
                               [current_node.y, current_node.parent.y],
                               'r-', linewidth=2)
            frame.append(line)
        
        self.frames.append(frame)
    
    def add_apf_frame(self, current_path):
        frame = []
        # Plot current refined path
        if len(current_path) > 1:
            path_x, path_y = zip(*current_path)
            line, = self.ax.plot(path_x, path_y, 'g-', linewidth=2)
            frame.append(line)
        
        self.frames.append(frame)
    
    def _animate(self, i):
        # Remove previous frame
        if i > 0:
            for line in self.frames[i-1]:
                line.remove()
        
        # Add current frame
        return self.frames[i]
    
    def save_animation(self, filename='Figure/hybrid_planning.gif', fps=10):
        anim = animation.ArtistAnimation(self.fig, self.frames, interval=1000/fps,
                                       blit=True, repeat=True)
        anim.save(filename, writer='pillow')
        plt.close()