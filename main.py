import os
import time
import random
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon

from planner.apf_local_planner import APFLocalPlanner
from planner.hybrid_planner import HybridPlanner
from planner.rrt_planner import RRTPlanner
from utils.animation import HybridPlannerAnimation


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
    # === Opzioni di esecuzione ===
    enable_anim = True            # animazione disattivata per velocità
    enable_potential_plot = True # campo potenziale disattivato inizialmente

    # Riproducibilità 
    random.seed(0)
    np.random.seed(0)

    # Cartella per output grafici/animazioni
    os.makedirs("figure", exist_ok=True)

    # === Parametri planner CORRETTI PER BILANCIAMENTO FORZE ===
    k_att = 10.0         # Aumentato per bilanciare le repulsive
    k_rep = 30.0         # MOLTO ridotto - era troppo forte!
    rho_0 = 8.0          # Ridotto per limitare l'influenza degli ostacoli
    local_step_size = 0.01 # Ridotto per movimenti più precisi
    delta_0 = 1.5        # Distanza di sicurezza ridotta

    hybrid_planner = HybridPlanner(k_att, k_rep, rho_0, local_step_size, delta_0)

    # === Ambiente ===
    obstacles = [
        create_obstacle((30, 30), 10, 10),
        create_obstacle((60, 60), 15, 15),
        create_obstacle((40, 70), 12, 12)
    ]

    # Start/Goal
    start_point = Point(10, 10)
    goal_point = Point(60, 90)

    # RRT con meno iterazioni per test rapidi
    rrt_planner = RRTPlanner(max_iterations=500)  # RIDOTTO da 1000

    # Animazione (creata solo se serve)
    animation = HybridPlannerAnimation(obstacles, start_point, goal_point) if enable_anim else None

    print("Starting planning...")
    t0 = time.time()

    # === Global path (RRT) ===
    print("RRT phase...")
    t_rrt_start = time.time()
    global_path = rrt_planner.plan_path(start_point, goal_point, obstacles, animation)
    t_rrt = time.time() - t_rrt_start
    
    if global_path is None:
        print("Failed to find a valid path using RRT")
        return

    print(f"RRT completed in {t_rrt:.2f}s, path length: {len(global_path)}")

    # === Refinement (APF) ===
    print("APF refinement phase...")
    t_apf_start = time.time()
    refined_path = hybrid_planner.refine_path(
        global_path, obstacles, 
        max_local_steps=50,  # RIDOTTO da 100 (default era molto più alto)
        animation=animation
    )
    t_apf = time.time() - t_apf_start
    print(f"APF completed in {t_apf:.2f}s, refined path length: {len(refined_path)}")

    # === Animazione (opzionale) ===
    if enable_anim and animation is not None:
        print("Saving animation...")
        animation.save_animation('figure/hybrid_planning.gif')

    # === Visualizzazione finale del percorso  ===
    print("Creating path visualization...")
    hybrid_planner.visualize_hybrid_path(
        global_path, refined_path, obstacles, start_point, goal_point
    )

    # === Campo potenziale (solo se esplicitamente richiesto) ===
    if enable_potential_plot:
        print("Computing potential field (this may take time)...")
        hybrid_planner.apf_planner.visualize_potential_field(
            start_point, goal_point, obstacles,
            x_range=(0, 100), y_range=(0, 100),
            resolution=30  
        )

    dt = time.time() - t0
    print(f"\n=== TIMING SUMMARY ===")
    print(f"RRT: {t_rrt:.2f}s")
    print(f"APF: {t_apf:.2f}s")
    print(f"Total: {dt:.2f}s")


if __name__ == '__main__':
    main()