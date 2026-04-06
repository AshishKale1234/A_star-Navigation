# TurtleBot4 Autonomous Navigation with A* Path Planning

Autonomous navigation system for the TurtleBot4 using custom SLAM mapping, A* path planning, and a pure-pursuit-style path follower — built entirely from scratch in ROS 2 Humble without Nav2.


---

## Overview

This project implements a full autonomous navigation pipeline for a differential-drive mobile robot:

1. **SLAM Mapping** — Generate an occupancy grid map of the environment using `slam_toolbox`, both in simulation (Gazebo) and on physical hardware (TurtleBot4).
2. **A\* Path Planning** — Load the occupancy grid, inflate obstacles by the robot's footprint radius, build a graph of free cells, and compute shortest paths using the A* algorithm with Euclidean heuristic and `heapq` priority queue.
3. **Path Following** — A lookahead-based controller that publishes `cmd_vel` commands to drive the robot through the planned waypoints with velocity smoothing.
4. **Hardware Deployment** — Localize on a pre-built map using AMCL and navigate the physical TurtleBot4 through a real classroom environment.

A companion Jupyter Notebook (`navigation_astar_f24.ipynb`) walks through BFS, Dijkstra, and A* on the same map for comparison.

## Demo

https://www.dropbox.com/scl/fi/15r06xz7ezlj5jztfyu6d/A_star_navigation.mp4?rlkey=hhy2typpejguz4yrl39t5catd&st=llbs0ma4&dl=0

## Repository Structure

```
.
├── task_4/                         # ROS 2 ament_python package
│   ├── task_4/
│   │   ├── __init__.py
│   │   └── auto_navigator.py       # Main navigation node (A*, path follower)
│   ├── launch/
│   │   └── gen_sync_map_launch.py   # SLAM mapping launch file
│   ├── maps/
│   │   ├── classroom_map.pgm        # Hardware occupancy grid image
│   │   ├── classroom_map.yaml       # Hardware map metadata
│   │   ├── sync_classroom_map.pgm   # Simulation occupancy grid image
│   │   └── sync_classroom_map.yaml  # Simulation map metadata
│   ├── rviz/
│   │   └── robot.rviz               # RViz configuration
│   ├── test/                        # ament linting tests
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── LICENSE
├── navigation_astar_f24.ipynb       # Jupyter Notebook: BFS, Dijkstra, A* comparison
├── .gitignore
└── README.md
```

## Prerequisites

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble Hawksbill |
| Simulator | Gazebo Classic 11.10 |
| Robot model (sim) | TurtleBot3 Waffle |
| Robot model (hw) | TurtleBot4 |
| Python | 3.10+ |

### Python Dependencies (auto-installed with ROS 2)

- `rclpy`, `nav_msgs`, `geometry_msgs`, `std_msgs`
- `tf_transformations`
- `numpy`, `Pillow`, `PyYAML`
- `heapq` (stdlib)

### Additional ROS 2 Packages

```bash
sudo apt install ros-humble-turtlebot3-teleop \
                 ros-humble-slam-toolbox \
                 ros-humble-navigation2 \
                 ros-humble-turtlebot4-navigation \
                 ros-humble-turtlebot4-viz
```

## Setup

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src    # or your preferred workspace
git clone https://github.com/<your-username>/turtlebot4-autonomous-navigation.git
```

### 2. Set Up the Simulation Workspace

Follow the [sim_ws setup instructions](https://github.com/Purdue-ME597/sim_ws) to install the TurtleBot3 Gazebo simulation environment, then:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 3. Copy the Package into Your Workspace

```bash
cp -r turtlebot4-autonomous-navigation/task_4 ~/ros2_ws/src/
```

### 4. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select task_4
source install/local_setup.bash
```

## Usage

### Mapping (Simulation)

Generate a map by teleoperating the robot through the simulated house:

```bash
# Terminal 1 — Launch Gazebo + SLAM
ros2 launch turtlebot3_gazebo mapper.launch.py

# Terminal 2 — Teleoperate the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once the map looks complete in RViz, save it:

```bash
ros2 run nav2_map_server map_saver_cli -f sync_classroom_map
```

### Mapping (Hardware)

```bash
# Launch SLAM + RViz on the physical TurtleBot4
ros2 launch task_4 gen_sync_map_launch.py namespace:=/robot

# Teleoperate in a separate terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/robot/cmd_vel
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f classroom_map \
    --ros-args -p map_subscribe_transient_local:=true -r __ns:=/robot
```

### Autonomous Navigation (Simulation)

```bash
# Launch the navigator (loads map, starts auto_navigator node)
ros2 launch turtlebot3_gazebo navigator.launch.py
```

Then set a **2D Goal Pose** in RViz. The robot will plan a path with A* and autonomously navigate to the goal.

### Autonomous Navigation (Hardware)

```bash
ros2 launch task_4 turtlebot4_navigator.launch.py
```

1. Set the **Initial Pose** in RViz to match the robot's physical position.
2. Align the LIDAR scan with the map walls.
3. Set a **2D Goal Pose** — the robot navigates autonomously.

## How It Works

### A* Path Planner

The `auto_navigator.py` node implements A* from scratch:

- **Map loading** — Reads the `.yaml` + `.pgm` occupancy grid, converts pixel intensities to free / occupied cells using the trinary threshold model.
- **Obstacle inflation** — Dilates occupied cells by the robot radius + safety margin (~0.28 m total) using a circular structuring element, preventing the planner from routing too close to walls.
- **Graph construction** — Each free cell becomes a node connected to its 8 neighbors (cardinal cost = 1.0, diagonal cost = √2).
- **A\* search** — Uses `heapq` for O(log n) priority queue operations with Euclidean distance heuristic. Snap-to-nearest-free handles cases where the start/goal falls on an inflated cell.

### Path Follower

A lookahead-based controller:

- Finds the first waypoint beyond a **lookahead distance** (0.30 m) from the robot.
- Computes heading error and applies proportional angular control.
- If heading error exceeds a threshold, the robot **rotates in place** before driving forward.
- Linear and angular velocities are **exponentially smoothed** (α = 0.45) for stable motion.

### Tunable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.19 m | Physical robot radius for inflation |
| `extra_inflation` | 0.09 m | Additional safety margin around obstacles |
| `lookahead` | 0.30 m | Path follower lookahead distance |
| `goal_tol` | 0.12 m | Distance threshold to consider goal reached |
| `max_lin` | 0.07 m/s | Maximum linear velocity |
| `max_ang` | 0.6 rad/s | Maximum angular velocity |

> **Note:** The `map_yaml` path in `auto_navigator.py` is currently hardcoded. Update it to match your workspace before running.

## Jupyter Notebook

`navigation_astar_f24.ipynb` provides an interactive comparison of three path-planning algorithms on the classroom map:

- **BFS** — Unweighted shortest path
- **Dijkstra** — Weighted shortest path
- **A\*** — Weighted + heuristic-guided

Open in [Google Colab](https://colab.research.google.com/) or locally with Jupyter. The notebook visualizes planned paths overlaid on the inflated occupancy grid.

## Acknowledgments

- ME 597 course staff at Purdue University for the lab framework and skeleton code
- [TurtleBot4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/) for hardware reference
- [Purdue ME597 sim_ws](https://github.com/Purdue-ME597/sim_ws) for the Gazebo simulation environment
