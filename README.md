# 🧭 Hybrid Path Planning (RRT + APF)

Repository for a hybrid path planning project that combines **Rapidly-exploring Random Trees (RRT)** for global exploration with **Artificial Potential Fields (APF)** for local path refinement.  
This approach leverages the strengths of both planners to generate efficient and smooth trajectories for autonomous navigation.

<details>
<summary>🚀 How to run my project? </summary>

**Prerequisites:**  

```bash
pip install numpy matplotlib shapely
```

**Run the project:**  

```bash
python main.py
```

This will:
- Generate a global path using RRT
- Refine it with APF
- Visualize the process and save:
  - The environment with obstacles
  - The RRT initial path
  - The smoothed APF path
  - A GIF animation of the planning process
  - The potential field heatmap

</details>