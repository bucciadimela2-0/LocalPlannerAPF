import numpy as np
from shapely.geometry import LineString, Point, Polygon
import matplotlib.pyplot as plt

class APFLocalPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, rho_0=10.0, delta=1.0):
        self.k_att = k_att
        self.k_rep = k_rep
        self.rho_0 = rho_0
        self.delta = delta
        # Cache per ottimizzazioni
        self._obstacle_bounds = {}
    
    def _get_obstacle_bounds(self, obstacle):
        """Cache dei bounds degli ostacoli per filtraggio rapido."""
        if id(obstacle) not in self._obstacle_bounds:
            bounds = obstacle.bounds  # (minx, miny, maxx, maxy)
            self._obstacle_bounds[id(obstacle)] = bounds
        return self._obstacle_bounds[id(obstacle)]
    
    def _quick_distance_check(self, point, obstacle):
        """Controllo rapido della distanza usando bounding box."""
        bounds = self._get_obstacle_bounds(obstacle)
        minx, miny, maxx, maxy = bounds
        
        # Distanza approssimata dalla bounding box
        dx = max(0, max(minx - point.x, point.x - maxx))
        dy = max(0, max(miny - point.y, point.y - maxy))
        approx_dist = np.sqrt(dx*dx + dy*dy)
        
        return approx_dist

    def attractive_force(self, current_pos, goal_pos):
        """Calculate attractive force towards the goal."""
        diff = np.array([goal_pos.x - current_pos.x, goal_pos.y - current_pos.y])
        distance = np.linalg.norm(diff)
        return self.k_att * (diff / (distance + 1e-4))
    
    def repulsive_force(self, current_pos, obstacles):
        """Calculate repulsive force from obstacles with optimizations."""
        total_force = np.zeros(2)
        current_point = Point(current_pos.x, current_pos.y)
        
        for obstacle in obstacles:
            # Prima un controllo rapido
            approx_dist = self._quick_distance_check(current_point, obstacle)
            
            # Se la distanza approssimata è maggiore di rho_0, salta
            if approx_dist > self.rho_0:
                continue
                
            # Solo ora calcola la distanza precisa
            distance = current_point.distance(obstacle)
            
            # Only apply repulsive force if within influence range
            if distance < self.rho_0:
                # Get closest point on obstacle using Shapely
                closest_point = obstacle.exterior.interpolate(obstacle.exterior.project(current_point))
                
                # Calculate direction from obstacle to current position
                diff = np.array([current_pos.x - closest_point.x, current_pos.y - closest_point.y])
                diff_norm = np.linalg.norm(diff)
                epsilon = 1e-4
                
                if distance > self.delta:
                    force = self.k_rep * ((1.0 / (distance - self.delta)) - (1.0 / (self.rho_0 - self.delta))) * \
                        (1.0 / ((distance - self.delta) ** 2)) * (diff / (diff_norm + epsilon))
                else:
                    force = self.k_rep * (1.0 / epsilon**2) * (diff / (diff_norm + epsilon))
            
                total_force += force
        
        return total_force
    
    def get_motion_direction(self, current_pos, goal_pos, obstacles, debug=False):
        """Get the motion direction based on total force."""
        f_att = self.attractive_force(current_pos, goal_pos)
        f_rep = self.repulsive_force(current_pos, obstacles)
        total_force = f_att + f_rep
        
        if debug:
            print(f"Pos: ({current_pos.x:.1f}, {current_pos.y:.1f}) | "
                  f"F_att: ({f_att[0]:.2f}, {f_att[1]:.2f}) | "
                  f"F_rep: ({f_rep[0]:.2f}, {f_rep[1]:.2f}) | "
                  f"F_total: ({total_force[0]:.2f}, {total_force[1]:.2f})")
        
        # Normalize force vector
        force_norm = np.linalg.norm(total_force)
        if force_norm < 1e-8:
            # spintarella casuale per uscire dallo stallo
            jitter = np.random.randn(2)
            jitter /= (np.linalg.norm(jitter) + 1e-8)
            if debug:
                print("Force too small, using jitter")
            return jitter
        return total_force / force_norm
    
    def plan_local_path(self, start_pos, goal_pos, obstacles, step_size=1.0, max_steps=200):
        """Generate local path using APF with improved stall detection."""
        path = [(start_pos.x, start_pos.y)]
        current_pos = Point(start_pos.x, start_pos.y)
        goal_reached = False
        step_count = 0
        
        # Controllo di convergenza migliorato
        prev_positions = []
        stall_threshold = step_size * 0.3  # Più dinamico basato su step_size
        min_steps_before_stall_check = 15  # Aspetta più step prima di controllare stallo
        
        while not goal_reached and step_count < max_steps:
            # Debug per i primi 10 step
            debug_this_step = (step_count < 10)
            direction = self.get_motion_direction(current_pos, goal_pos, obstacles, debug=debug_this_step)
            
            # Update position
            new_x = current_pos.x + step_size * direction[0]
            new_y = current_pos.y + step_size * direction[1]
            new_pos = Point(new_x, new_y)
            
            segment = LineString([(current_pos.x, current_pos.y), (new_x, new_y)])

            # Check if new position is collision-free
            collision_free = True
            for obs in obstacles:
                if obs.contains(new_pos) or segment.intersects(obs):
                    collision_free = False
                    break
            
            if collision_free:
                current_pos = new_pos
                path.append((new_x, new_y))
                
                # Check if goal is reached
                dist_to_goal = np.sqrt((goal_pos.x - new_x)**2 + (goal_pos.y - new_y)**2)
                if dist_to_goal < step_size * 2.0:  # Più tollerante
                    goal_reached = True
                    print(f"APF reached goal at step {step_count}")
                
                # Controllo stallo - solo dopo un numero minimo di step
                if step_count > min_steps_before_stall_check:
                    prev_positions.append((new_x, new_y))
                    if len(prev_positions) > 10:  # Finestra più ampia
                        prev_positions.pop(0)
                        if len(prev_positions) == 10:
                            # Controlla se negli ultimi 10 step si è mosso poco
                            recent_movement = np.sqrt((prev_positions[-1][0] - prev_positions[0][0])**2 + 
                                                    (prev_positions[-1][1] - prev_positions[0][1])**2)
                            if recent_movement < stall_threshold:
                                print(f"APF stalled at step {step_count} (movement: {recent_movement:.2f} < {stall_threshold:.2f})")
                                break
            else:
                # Se c'è collisione, prova a fare un piccolo jitter
                if step_count % 10 == 0:  # Ogni 10 step in collisione
                    jitter = np.random.randn(2) * step_size * 0.3
                    current_pos = Point(current_pos.x + jitter[0], current_pos.y + jitter[1])
            
            step_count += 1
        
        print(f"APF finished: steps={step_count}, goal_reached={goal_reached}, path_length={len(path)}")
        return path, goal_reached
    
    def visualize_potential_field(self, start_pos, goal_pos, obstacles, x_range=(0, 100), y_range=(0, 100), resolution=50):
        """Visualize potential field with reduced resolution for speed."""
        
        x = np.linspace(x_range[0], x_range[1], resolution)
        y = np.linspace(y_range[0], y_range[1], resolution)
        X, Y = np.meshgrid(x, y)

        U = np.zeros_like(X)
        V = np.zeros_like(Y)

        print(f"Computing potential field on {resolution}x{resolution} grid...")
        
        for i in range(X.shape[0]):
            if i % 10 == 0:  # Progress indicator
                print(f"Row {i}/{X.shape[0]}")
                
            for j in range(X.shape[1]):
                current_pos = Point(X[i,j], Y[i,j])
                
                # Verifica se il punto è all'interno di qualsiasi ostacolo
                inside_obstacle = False
                for obstacle in obstacles:
                    if obstacle.contains(current_pos):
                        inside_obstacle = True
                        break
                
                # Calcola il potenziale solo per punti fuori dagli ostacoli
                if not inside_obstacle:
                    direction = self.get_motion_direction(current_pos, goal_pos, obstacles)
                    U[i,j] = direction[0]
                    V[i,j] = direction[1]
        
        plt.figure(figsize=(10, 8))
        plt.streamplot(X, Y, U, V, density=1.0) 
        
        # Plot obstacles
        for obstacle in obstacles:
            plt.fill(*zip(*obstacle.exterior.coords), color='gray', alpha=0.3)
        
        plt.plot(start_pos.x, start_pos.y, 'go', markersize=10, label='Start')
        plt.plot(goal_pos.x, goal_pos.y, 'ro', markersize=10, label='Goal')
        plt.grid(True)
        plt.legend()
        plt.title('Artificial Potential Field (Optimized)')
        plt.savefig(fname='figure/potential_field.png', dpi=150)
        plt.show()