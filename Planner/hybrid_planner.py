import numpy as np
from shapely.geometry import Point

from Planner.apf_local_planner import APFLocalPlanner


class HybridPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, rho_0=10.0, local_step_size=1.0):
        self.apf_planner = APFLocalPlanner(k_att, k_rep, rho_0)
        self.local_step_size = local_step_size

    def refine_path(self, global_path, obstacles, max_local_steps=100, animation=None):
        """Refine the global path using APF for local planning."""
        refined_path = []
        path_points = [Point(x, y) for x, y in global_path]
        
        if animation:
            animation.add_apf_frame(global_path)

        for i in range(len(path_points) - 1):
            start = path_points[i]
            goal = path_points[i + 1]
            
            # Use APF to plan local path between consecutive global path points
            local_path, reached = self.apf_planner.plan_local_path(
                start, goal, obstacles, 
                step_size=self.local_step_size,
                max_steps=max_local_steps
            )
            
            # Add local path points to refined path
            if i == 0:
                refined_path.extend(local_path)
            else:
                # Skip first point as it's the same as last point of previous segment
                refined_path.extend(local_path[1:])
                
                if animation:
                    animation.add_apf_frame(refined_path)

        return refined_path

    def visualize_hybrid_path(self, global_path, refined_path, obstacles, start_point, goal_point):
        """Visualize both global and refined paths."""
        import matplotlib.pyplot as plt

        plt.figure(figsize=(12, 8))
        
        # Plot obstacles
        for obstacle in obstacles:
            plt.fill(*zip(*obstacle.exterior.coords), color='gray', alpha=0.3)
        
        # Plot global path
        global_x, global_y = zip(*global_path)
        plt.plot(global_x, global_y, 'b--', linewidth=2, label='Global Path (RRT)')
        
        # Plot refined path
        refined_x, refined_y = zip(*refined_path)
        plt.plot(refined_x, refined_y, 'g-', linewidth=2, label='Refined Path (APF)')
        
        # Plot start and goal points
        plt.plot(start_point.x, start_point.y, 'go', markersize=10, label='Start')
        plt.plot(goal_point.x, goal_point.y, 'ro', markersize=10, label='Goal')
        
        plt.grid(True)
        plt.legend()
        plt.title('Hybrid Path Planning (RRT + APF)')
        plt.show()