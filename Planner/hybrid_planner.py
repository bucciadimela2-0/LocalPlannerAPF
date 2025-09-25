import numpy as np
from shapely.geometry import LineString, Point
from planner.apf_local_planner import APFLocalPlanner


class HybridPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, rho_0=10.0, local_step_size=1.0, delta_0=1.0):
        self.apf_planner = APFLocalPlanner(k_att, k_rep, rho_0, delta_0)
        self.local_step_size = local_step_size

    def refine_path(self, global_path, obstacles, max_local_steps=50, animation=None):
        """Refine the global path using hybrid approach with optimizations."""
        if len(global_path) < 2:
            return global_path
        
        # Initialize with start point
        refined_path = [global_path[0]]
        
        # Set up starting point and final goal
        current_pos = Point(global_path[0][0], global_path[0][1])
        final_goal = Point(global_path[-1][0], global_path[-1][1])
        
        # Find all intermediate waypoints from RRT
        waypoints = [Point(x, y) for x, y in global_path[1:]]
        
        # Target index in waypoints
        target_idx = 0
        
        # Cache pre-computed obstacle buffers
        obstacle_buffers = [obs.buffer(self.apf_planner.rho_0) for obs in obstacles]
        
        while target_idx < len(waypoints):
            # Current target waypoint
            target = waypoints[target_idx]
            
            # Check if we're in an obstacle influence area (using cached buffers)
            in_obstacle_influence = False
            segment = LineString([(current_pos.x, current_pos.y), (target.x, target.y)])
            
            for obstacle_buffer in obstacle_buffers:
                if segment.intersects(obstacle_buffer):
                    in_obstacle_influence = True
                    break
            
            if in_obstacle_influence:
                # We're near an obstacle, use APF to plan a path towards the target
                local_path, reached = self.apf_planner.plan_local_path(
                    current_pos, target, obstacles,
                    step_size=self.local_step_size,
                    max_steps=max_local_steps
                )
                
                if len(local_path) > 1:
                    # Add all points from APF except the first (already in refined_path)
                    for j in range(1, len(local_path)):
                        refined_path.append(local_path[j])
                    
                    # Update current position to the last point from APF
                    current_pos = Point(local_path[-1][0], local_path[-1][1])
                    
                    # Determine if we should move to the next waypoint
                    distance_to_target = current_pos.distance(Point(target.x, target.y))
                    if distance_to_target < 3.0 or reached:  # Un po' più tollerante
                        target_idx += 1
                    # Otherwise, keep the same target for next iteration
                else:
                    # APF failed, directly go to target (fallback)
                    refined_path.append((target.x, target.y))
                    current_pos = target
                    target_idx += 1
            else:
                # No obstacle influence, direct path is safe
                refined_path.append((target.x, target.y))
                current_pos = target
                target_idx += 1
            
            if animation:
                animation.add_apf_frame(refined_path)
        
        return refined_path
     
    def visualize_hybrid_path(self, global_path, refined_path, obstacles, start_point, goal_point):
        """Visualize both global and refined paths with optimizations."""
        import matplotlib.pyplot as plt

        plt.figure(figsize=(12, 8))
        
        # Plot obstacles
        for obstacle in obstacles:
            x_coords, y_coords = obstacle.exterior.xy
            plt.fill(x_coords, y_coords, color='gray', alpha=0.3)
        
        # Plot global path
        if len(global_path) > 1:
            global_x, global_y = zip(*global_path)
            plt.plot(global_x, global_y, 'b--', linewidth=2, label='Global Path (RRT)', alpha=0.7)
            plt.scatter(global_x[1:-1], global_y[1:-1], c='blue', s=30, alpha=0.5, zorder=3)
        
        # Plot refined path
        if len(refined_path) > 1:
            refined_x, refined_y = zip(*refined_path)
            plt.plot(refined_x, refined_y, 'g-', linewidth=2, label='Refined Path (APF)')
        
        # Plot start and goal points
        plt.plot(start_point.x, start_point.y, 'go', markersize=12, label='Start', zorder=5)
        plt.plot(goal_point.x, goal_point.y, 'ro', markersize=12, label='Goal', zorder=5)
        
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title('Hybrid Path Planning (RRT + APF)')
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(fname='figure/hybrid_path.png', dpi=150, bbox_inches='tight')
        plt.show(block=False)  # Non blocca l'esecuzione
        plt.pause(0.1)  # Breve pausa per rendering