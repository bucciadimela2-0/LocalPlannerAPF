import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Point, Polygon

from Planner.apf_local_planner import APFLocalPlanner
from Planner.hybrid_planner import HybridPlanner
from Planner.rrt_planner import RRTPlanner
from Utils.animation import HybridPlannerAnimation


def create_obstacle(center, width, height):
    """Create a rectangular obstacle."""
    x, y = center
    vertices = [
        (x - width/2, y - height/2),
        (x + width/2, y - height/2),
        (x + width/2, y + height/2),
        (x - width/2, y + height/2),
        (x - width/2, y - height/2)
    ]
    return Polygon(vertices)

def main():
    # Initialize planners
    k_att = 100.0  # Attractive force gain
    k_rep = 1000.0  # Repulsive force gain
    rho_0 = 50.0  # Influence range of obstacles
    local_step_size = 2.0  # Step size for local planning
    
    hybrid_planner = HybridPlanner(k_att, k_rep, rho_0, local_step_size)
    
    # Create environment
    obstacles = [
        create_obstacle((30, 30), 10, 10),
        create_obstacle((60, 60), 15, 15),
        create_obstacle((40, 70), 12, 12)
    ]
    
    # Define start and goal points
    start_point = Point(10, 10)
    goal_point = Point(90, 90)
    
    # Initialize RRT planner
    rrt_planner = RRTPlanner(max_iterations=1000)
    
    # Create animation
    animation = HybridPlannerAnimation(obstacles, start_point, goal_point)
    
    # Generate global path using RRT with animation
    global_path = rrt_planner.plan_path(start_point, goal_point, obstacles, animation)
    if global_path is None:
        print("Failed to find a valid path using RRT")
        return
    
    # Refine path using APF with animation
    refined_path = hybrid_planner.refine_path(global_path, obstacles, animation=animation)
    
    # Save the animation
    animation.save_animation('Figure/hybrid_planning.gif')
    
    # Visualize final results
    hybrid_planner.visualize_hybrid_path(
        global_path, refined_path, obstacles, start_point, goal_point
    )
    
    # Visualize potential field
    hybrid_planner.apf_planner.visualize_potential_field(
        start_point, goal_point, obstacles,
        x_range=(0, 100), y_range=(0, 100)
    )

if __name__ == '__main__':
    main()