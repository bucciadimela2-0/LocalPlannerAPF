import numpy as np
import random
from shapely.geometry import Point, Polygon, LineString

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTPlanner:
    def __init__(self, max_iterations=1000):
        self.max_iterations = max_iterations

    def generate_random_point(self, min_xy=0, max_xy=100):
        x = random.uniform(min_xy, max_xy)
        y = random.uniform(min_xy, max_xy)
        return Point(x, y)

    def distance(self, node_x, node_y, point_x, point_y):
        return np.sqrt((node_x - point_x)**2 + (node_y - point_y)**2)

    def find_nearest_node(self, tree, random_point_x, random_point_y):
        min_dist = float('inf')
        nearest_node = None
        for node in tree:
            dist = self.distance(node.x, node.y, random_point_x, random_point_y)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def is_collision_free(self, point, obstacles):
        point_shapely = Point(point.x, point.y)
        for obstacle in obstacles:
            if obstacle.contains(point_shapely):
                return False
        return True

    def is_path_collision_free(self, start_point, end_point, obstacles):
        path_line = LineString([(start_point.x, start_point.y), (end_point.x, end_point.y)])
        for obstacle in obstacles:
            if path_line.intersects(obstacle):
                return False
        return True

    def choose_new_point(self, random_point, nearest_node, obstacles, step_size=5.0):
        # Calculate direction vector
        dx = random_point.x - nearest_node.x
        dy = random_point.y - nearest_node.y
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist == 0:
            return None

        # Scale to step size
        dx = dx / dist * min(step_size, dist)
        dy = dy / dist * min(step_size, dist)
        
        # New point coordinates
        new_x = nearest_node.x + dx
        new_y = nearest_node.y + dy
        new_point = Point(new_x, new_y)

        # Check if the new point and path to it are collision-free
        if self.is_collision_free(new_point, obstacles) and \
           self.is_path_collision_free(Point(nearest_node.x, nearest_node.y), new_point, obstacles):
            return (new_x, new_y)
        return None

    def plan_path(self, start_point, goal_point, obstacles, animation=None):
        tree = [Node(start_point.x, start_point.y)]
        goal_node = Node(goal_point.x, goal_point.y)
        
        if animation:
            animation.add_rrt_frame(tree)

        for i in range(self.max_iterations):
            if i % 10 == 0:  # Bias towards goal
                random_point = Point(goal_point.x, goal_point.y)
            else:
                random_point = self.generate_random_point()

            nearest_node = self.find_nearest_node(tree, random_point.x, random_point.y)
            new_point = self.choose_new_point(random_point, nearest_node, obstacles)

            if new_point is not None:
                new_node = Node(new_point[0], new_point[1])
                new_node.parent = nearest_node
                tree.append(new_node)
                
                if animation:
                    animation.add_rrt_frame(tree, new_node)

                # Check if we can connect to goal
                if self.distance(new_node.x, new_node.y, goal_point.x, goal_point.y) < 5.0 and \
                   self.is_path_collision_free(Point(new_node.x, new_node.y), goal_point, obstacles):
                    goal_node.parent = new_node
                    tree.append(goal_node)
                    return self._extract_path(goal_node)

        return None

    def _extract_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return list(reversed(path))