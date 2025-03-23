import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Point, Polygon


class APFLocalPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, rho_0=10.0):
        self.k_att = k_att  # Attractive potential gain
        self.k_rep = k_rep  # Repulsive potential gain
        self.rho_0 = rho_0  # Influence range of obstacles
        
    def attractive_force(self, current_pos, goal_pos):
        """Calculate attractive force towards the goal."""
        diff = np.array([goal_pos.x - current_pos.x, goal_pos.y - current_pos.y])
        distance = np.linalg.norm(diff)
        return self.k_att * diff
    
    def repulsive_force(self, current_pos, obstacles):
        """Calculate repulsive force from obstacles."""
        total_force = np.zeros(2)
        current_point = Point(current_pos.x, current_pos.y)
        
        for obstacle in obstacles:
            distance = current_point.distance(obstacle)
            
            if distance < self.rho_0:
                # Get closest point on obstacle
                boundary_points = list(obstacle.exterior.coords)
                min_dist = float('inf')
                closest_point = None
                
                for point in boundary_points:
                    dist = np.sqrt((point[0] - current_pos.x)**2 + (point[1] - current_pos.y)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_point = point
                
                # Calculate repulsive force direction
                diff = np.array([current_pos.x - closest_point[0], 
                                current_pos.y - closest_point[1]])
                diff_norm = np.linalg.norm(diff)
                epsilon = 1e-5  # Small value to prevent division by zero
                if diff_norm > epsilon:
                    force = (self.k_rep * (1/(distance + epsilon) - 1/self.rho_0) * 
                            (1/(distance + epsilon)**2) * (diff/diff_norm))
                    total_force += force
        
        return total_force
    
    def get_motion_direction(self, current_pos, goal_pos, obstacles):
        """Get the motion direction based on total force."""
        f_att = self.attractive_force(current_pos, goal_pos)
        f_rep = self.repulsive_force(current_pos, obstacles)
        total_force = f_att + f_rep
        
        # Normalize force vector
        force_norm = np.linalg.norm(total_force)
        if force_norm > 0:
            return total_force / force_norm
        return total_force
    
    def plan_local_path(self, start_pos, goal_pos, obstacles, step_size=1.0, max_steps=1000):
        """Generate local path using APF."""
        path = [(start_pos.x, start_pos.y)]
        current_pos = Point(start_pos.x, start_pos.y)
        goal_reached = False
        step_count = 0
        
        while not goal_reached and step_count < max_steps:
            direction = self.get_motion_direction(current_pos, goal_pos, obstacles)
            
            # Update position
            new_x = current_pos.x + step_size * direction[0]
            new_y = current_pos.y + step_size * direction[1]
            new_pos = Point(new_x, new_y)
            
            # Check if new position is collision-free
            if all(not obs.contains(new_pos) for obs in obstacles):
                current_pos = new_pos
                path.append((new_x, new_y))
                
                # Check if goal is reached
                if np.sqrt((goal_pos.x - new_x)**2 + (goal_pos.y - new_y)**2) < step_size:
                    goal_reached = True
            
            step_count += 1
        
        return path, goal_reached
    
    def visualize_potential_field(self, start_pos, goal_pos, obstacles, x_range=(0, 100), y_range=(0, 100)):
        """Visualize the potential field."""
        x = np.linspace(x_range[0], x_range[1], 100)
        y = np.linspace(y_range[0], y_range[1], 100)
        X, Y = np.meshgrid(x, y)
        
        U = np.zeros_like(X)
        V = np.zeros_like(Y)
        
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                current_pos = Point(X[i,j], Y[i,j])
                direction = self.get_motion_direction(current_pos, goal_pos, obstacles)
                U[i,j] = direction[0]
                V[i,j] = direction[1]
        
        plt.figure(figsize=(10, 8))
        plt.streamplot(X, Y, U, V, density=1.5)
        
        # Plot obstacles
        for obstacle in obstacles:
            plt.fill(*zip(*obstacle.exterior.coords), color='gray', alpha=0.3)
        
        plt.plot(start_pos.x, start_pos.y, 'go', markersize=10, label='Start')
        plt.plot(goal_pos.x, goal_pos.y, 'ro', markersize=10, label='Goal')
        plt.grid(True)
        plt.legend()
        plt.title('Artificial Potential Field')
        plt.savefig(fname = 'Figure/potential_field.png')
        plt.show()
