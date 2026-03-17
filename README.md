<div align="center">

<!-- HERO BANNER -->
<img src="docs/assets/banner.png" alt="SRM Autonomous Buggy Banner" width="100%"/>

<br/>

**AUTONOMOUS ELECTRIC CAMPUS BUGGY**

*Phase 1 — Software-in-the-Loop (SITL) Simulation*

---

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20Hawksbill-blue?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange?style=for-the-badge)](http://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python)](https://python.org)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-purple?style=for-the-badge&logo=ubuntu)](https://ubuntu.com)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

**SRM Institute of Science and Technology**  
Department of Artificial Intelligence and Machine Learning  
*Sprint: March 2026 · Phase 1 SITL · 6-Person Team*

</div>

---

## ⚡ What This Is

> **One command. One vision. Full autonomy.**

This repository contains the complete **Phase 1 Software-in-the-Loop (SITL)** simulation of the SRM Autonomous Electric Campus Buggy. A real autonomous navigation stack — not a tutorial, not a demo template — running *your* vehicle, *your* sensors, and *your* campus roads, all inside a computer before a single bolt is torqued on the real vehicle.

**What happens when you run `./run_demo.sh` →**

```
🌍 Gazebo opens  →  3D campus world  →  Buggy sits at START
🖥️  Terminal asks:  Select Destination: (A) Main Gate  (B) Library Block  (C) Admin Block
⌨️  You type: B
🧠  Dijkstra computes: START → HUB → Library Block  (40 m)
🚗  Buggy accelerates to 15 km/h autonomously
🛑  Detects obstacle at 1.5 m  →  Emergency Stop
✅  Obstacle cleared  →  5 clean LiDAR scans  →  Resumes
🏁  Arrives at Library Block  →  "Destination Reached!"
🔁  Waits for next destination  →  Repeat forever
```

---

## 📋 Table of Contents

- [Quick Start](#-quick-start)
- [System Architecture](#-system-architecture)
- [ROS 2 Node Reference](#-ros-2-node-reference)
- [Topic Registry](#-topic-registry)
- [Campus Map](#-campus-map)
- [Sensor Configuration](#-sensor-configuration)
- [Traceability Matrix](#-traceability-matrix)
- [Day-by-Day Schedule](#-day-by-day-schedule)
- [Known Pitfalls](#-known-pitfalls--fixes)
- [Phase 2 Roadmap](#-phase-2-roadmap)
- [Contributing Guidelines](#-contributing-guidelines)
- [Team](#-team)

---

## 🚀 Quick Start

### Prerequisites

```bash
# Confirm your Ubuntu version (must be 22.04)
lsb_release -a

# Install all required ROS 2 packages (one-shot command)
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rqt \
  ros-humble-rqt-graph \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-rosdep \
  tmux
```

### Clone & Build

```bash
# Clone this repository into your workspace
mkdir -p ~/srm_buggy_ws/src && cd ~/srm_buggy_ws/src
git clone https://github.com/SRM-Autonomous-Buggy/srm-autonomous-buggy.git .

# Source ROS 2 (add to ~/.bashrc permanently)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Build the workspace
cd ~/srm_buggy_ws
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash
```

### 🎬 Launch the Full Demo

```bash
# THE ONE COMMAND THAT RULES THEM ALL
./run_demo.sh
```

> The script will kill any stale Gazebo processes, source the workspace, start Gazebo with the SRM campus world, spawn the buggy, launch all ROS 2 nodes, open RViz2 with the pre-configured layout, and attach a tmux 4-pane dashboard — all automatically.

---

## 🏗️ System Architecture

```
╔══════════════════════════════════════════════════════════════════════╗
║                    SRM AUTONOMOUS BUGGY — SITL STACK                ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║   ┌─────────────┐   /scan        ┌──────────────────┐               ║
║   │   GAZEBO    │──────────────▶│ obstacle_detector │──┐            ║
║   │   LiDAR     │               └──────────────────┘  │ /obstacle_  ║
║   │   Camera    │   /ultrasonic  ┌──────────────────┐  │  detected   ║
║   │   IMU       │──────────────▶│ ultrasonic_monitor│──┤            ║
║   │   GPS       │               └──────────────────┘  │ /ultrasonic_║
║   │   Diff Drive│   /scan        ┌──────────────────┐  │   alert     ║
║   └─────────────┘──────────────▶│  crowd_detector   │──┤            ║
║          ▲                       └──────────────────┘  │ /crowd_     ║
║          │ /cmd_vel                                     │  detected   ║
║   ┌──────┴──────┐                ┌──────────────────┐  │            ║
║   │speed_       │◀──/buggy_state─│  STATE MACHINE   │◀─┘            ║
║   │controller   │                │   (Master FSM)   │               ║
║   └─────────────┘                └────────┬─────────┘               ║
║                                           │ /buggy_state             ║
║   ┌─────────────┐  /planned_path ┌────────▼─────────┐               ║
║   │ path_planner│──────────────▶│ waypoint_follower │               ║
║   │   + Dijkstra│                └──────────────────┘               ║
║   │  [TERMINAL] │                                                    ║
║   └─────────────┘                ┌──────────────────┐               ║
║                                  │  demo_visualizer  │──▶ RViz2     ║
║                                  └──────────────────┘               ║
╚══════════════════════════════════════════════════════════════════════╝
```

### Package Structure

```
srm_buggy_ws/
├── src/
│   ├── buggy_description/          # 🤖 Virtual Vehicle
│   │   ├── urdf/
│   │   │   ├── srm_buggy.xacro     # Base XACRO model
│   │   │   └── sensors.xacro       # Eight sensor link definitions
│   │   └── meshes/                 # STL/DAE mesh files
│   │
│   ├── buggy_bringup/              # 🚀 Launch & World
│   │   ├── launch/
│   │   │   ├── campus_world.launch.py
│   │   │   ├── buggy_spawn.launch.py
│   │   │   └── full_system.launch.py  ← THE MAIN LAUNCHER
│   │   ├── worlds/
│   │   │   └── srm_campus.world    # 50×50 m campus SDF
│   │   └── rviz/
│   │       └── srm_buggy_demo.rviz
│   │
│   ├── buggy_brain/                # 🧠 Intelligence Stack
│   │   └── buggy_brain/
│   │       ├── map_graph.py            # Campus graph + Dijkstra
│   │       ├── path_planner_node.py    # Terminal input → route
│   │       ├── waypoint_follower.py    # P-controller navigation
│   │       ├── obstacle_detector.py    # LiDAR perception (1.5 m)
│   │       ├── ultrasonic_monitor.py   # Corner guard (0.30 m)
│   │       ├── state_machine.py        # Master FSM (7 states)
│   │       ├── speed_controller.py     # /cmd_vel arbitrator
│   │       ├── crowd_detector.py       # LiDAR ray-density crowd
│   │       └── demo_visualizer.py      # RViz2 MarkerArray
│   │
│   └── docs/                       # 📄 Documentation
│       ├── implementation_plan.pdf
│       ├── traceability_matrix.md
│       └── assets/
│
├── run_demo.sh                     # 🎬 ONE-COMMAND LAUNCHER
└── README.md
```

---

## 🧠 ROS 2 Node Reference

| Node | File | Role | Pub Topics | Sub Topics |
|------|------|------|-----------|-----------|
| `path_planner_node` | `path_planner_node.py` | Terminal menu + Dijkstra | `/planned_path`, `/navigation_command` | — |
| `waypoint_follower_node` | `waypoint_follower.py` | P-controller 10 Hz | `/cmd_vel` | `/planned_path`, `/odom` |
| `obstacle_detector_node` | `obstacle_detector.py` | LiDAR safety (1.5 m) | `/obstacle_detected`, `/obstacle_direction` | `/scan` |
| `ultrasonic_monitor_node` | `ultrasonic_monitor.py` | Corner guard (0.30 m) | `/ultrasonic_alert` | `/ultrasonic/front,rear,left,right` |
| `crowd_detector_node` | `crowd_detector.py` | Ray-density crowd proxy | `/crowd_detected` | `/scan` |
| `state_machine_node` | `state_machine.py` | Master FSM (7 states) | `/buggy_state` | `/obstacle_detected`, `/ultrasonic_alert`, `/crowd_detected`, `/navigation_command` |
| `speed_controller_node` | `speed_controller.py` | Velocity arbitrator | `/cmd_vel` | `/buggy_state` |
| `demo_visualizer_node` | `demo_visualizer.py` | RViz2 markers | `/visualization_markers` | `/planned_path`, `/obstacle_direction` |

---

## 📡 Topic Registry

| ROS 2 Topic | Message Type | Hz | Source → Consumer |
|-------------|-------------|-----|-------------------|
| `/scan` | `sensor_msgs/LaserScan` | 10 | Gazebo LiDAR → `obstacle_detector`, `crowd_detector` |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 | Gazebo Camera → RViz2 |
| `/imu/data` | `sensor_msgs/Imu` | 100 | Gazebo IMU → *(Phase 2 fusion)* |
| `/gps/fix` | `sensor_msgs/NavSatFix` | 10 | Gazebo GPS → dashboard |
| `/ultrasonic/front` | `sensor_msgs/LaserScan` | 10 | Gazebo → `ultrasonic_monitor` |
| `/ultrasonic/rear` | `sensor_msgs/LaserScan` | 10 | Gazebo → `ultrasonic_monitor` |
| `/ultrasonic/left` | `sensor_msgs/LaserScan` | 10 | Gazebo → `ultrasonic_monitor` |
| `/ultrasonic/right` | `sensor_msgs/LaserScan` | 10 | Gazebo → `ultrasonic_monitor` |
| `/odom` | `nav_msgs/Odometry` | 50 | Diff Drive Plugin → `waypoint_follower` |
| `/cmd_vel` | `geometry_msgs/Twist` | 10 | `speed_controller` → Gazebo Diff Drive |
| `/obstacle_detected` | `std_msgs/Bool` | 10 | `obstacle_detector` → `state_machine` |
| `/obstacle_direction` | `std_msgs/Float32` | 10 | `obstacle_detector` → `demo_visualizer` |
| `/ultrasonic_alert` | `std_msgs/Bool` | 10 | `ultrasonic_monitor` → `state_machine` |
| `/crowd_detected` | `std_msgs/Bool` | 2 | `crowd_detector` → `state_machine` |
| `/buggy_state` | `std_msgs/String` | 10 | `state_machine` → `speed_controller` |
| `/planned_path` | `nav_msgs/Path` | on-demand | `path_planner` → `waypoint_follower` + RViz2 |
| `/navigation_command` | `std_msgs/String` | on-demand | `path_planner` → `state_machine` |
| `/visualization_markers` | `visualization_msgs/MarkerArray` | 5 | `demo_visualizer` → RViz2 |

---

## 🗺️ Campus Map

```text
  SRM Institute (A)
       (0, 50)
          │
          │  50 m
          │
    BUGGY_HUB ──────────────── SRM Hospital (B)
     (0, 0)        50 m            (50, 0)
          │
          │  50 m
          │
    SRM Temple (C)
      (0, -50)
```

| Node | Gazebo Coordinates | Destination Letter |
|------|-------------------|--------------------|
| **BUGGY_HUB** | `(0.0, 0.0)` | — |
| **SRM Institute (A)** | `(0.0, 50.0)` | `A` |
| **SRM Hospital (B)** | `(50.0, 0.0)` | `B` |
| **SRM Temple (C)** | `(0.0, -50.0)` | `C` |

**Dijkstra Sample Output:**
```text
BUGGY_HUB → A :  BUGGY_HUB  →  SRM_IST     (50.0 m)
BUGGY_HUB → B :  BUGGY_HUB  →  SRM_HOSP    (50.0 m)
BUGGY_HUB → C :  BUGGY_HUB  →  SRM_TEMPLE  (50.0 m)
```

---

## 🔬 Sensor Configuration

| Sensor | Link Name | XYZ Offset | Plugin | Topic | Real-World Proxy |
|--------|-----------|-----------|--------|-------|-----------------|
| LiDAR | `lidar_link` | `0.0, 0.0, 1.85` | `libgazebo_ros_ray_sensor.so` | `/scan` | Livox Mid-360 |
| Camera | `camera_link` | `1.2, 0.0, 1.20` | `libgazebo_ros_camera.so` | `/camera/image_raw` | Intel RealSense D435i |
| IMU | `imu_link` | `0.0, 0.0, 0.50` | `libgazebo_ros_imu_sensor.so` | `/imu/data` | BNO055 |
| GPS | `gps_link` | `-0.5, 0.0, 1.60` | `libgazebo_ros_gps_sensor.so` | `/gps/fix` | U-blox ZED-F9P |
| Ultrasonic FR | `ultrasonic_front_link` | `1.5, 0.0, 0.30` | `libgazebo_ros_ray_sensor.so` | `/ultrasonic/front` | HC-SR04 |
| Ultrasonic RR | `ultrasonic_rear_link` | `-1.5, 0.0, 0.30` | `libgazebo_ros_ray_sensor.so` | `/ultrasonic/rear` | HC-SR04 |
| Ultrasonic LT | `ultrasonic_left_link` | `0.0, 0.8, 0.30` | `libgazebo_ros_ray_sensor.so` | `/ultrasonic/left` | HC-SR04 |
| Ultrasonic RT | `ultrasonic_right_link` | `0.0, -0.8, 0.30` | `libgazebo_ros_ray_sensor.so` | `/ultrasonic/right` | HC-SR04 |

---

## 🔗 Traceability Matrix

Every threshold, every speed, every state transition is traced to the SRM Implementation Plan document.

| Code Component | Plan Section | What It Implements |
|---------------|-------------|-------------------|
| `map_graph.py` → `NODES` dict | §2.2 ODD Table | Named campus destinations |
| `map_graph.py` → `EDGES` dict | §2.2 | Campus road network topology |
| `map_graph.py` → `find_shortest_path()` | §5.2 Path Planning | Dijkstra shortest-route computation |
| `obstacle_detector.py` → `1.5 m` threshold | §5.1 | LiDAR emergency stop distance |
| `ultrasonic_monitor.py` → `0.30 m` threshold | §11.1 | Defence-in-depth corner guard |
| `state_machine.py` → `EMERGENCY_STOP` | §5.2.1 | FSM stop state transition |
| `state_machine.py` → 5-clear-reading resume | §5.2.1 | Safe-resume verification logic |
| `state_machine.py` → `CROWD_DETECTED` | §7.1 Level 3 | SAE Level 3 driver takeover |
| `speed_controller.py` → `4.17 m/s` | §2.2 ODD | Max speed 15 km/h enforcement |
| `crowd_detector.py` → 40-ray threshold | §7.1 / §8.2 | LiDAR ray-density crowd proxy |
| `lidar_link` XYZ `(0.0, 0.0, 1.85)` | §3.2 Sensor Table | Livox Mid-360 roof mount |
| `camera_link` XYZ `(1.2, 0.0, 1.20)` | §3.2 Sensor Table | RealSense D435i front mount |
| Gazebo LiDAR 360° @ 10 Hz | §4.1 | Mid-360 scan pattern simulation |
| `state_machine.py` (7-state FSM) | §5.2 FSM | Full behavioural state machine |
| `srm_campus.world` — HUB at `(0,0)` | §2.2 ODD Map | Physical road geometry |

---

## 🧪 Verification Commands

```bash
# Confirm all sensor topics are alive
ros2 topic list | grep -E "scan|camera|imu|gps|ultrasonic|odom|cmd_vel"

# Check LiDAR publishing at 10 Hz
ros2 topic hz /scan

# Manually drive the buggy
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once

# Trigger emergency stop manually
ros2 topic pub /obstacle_detected std_msgs/msg/Bool '{data: true}' --once

# View state machine output live
ros2 topic echo /buggy_state

# View TF tree
ros2 run tf2_tools view_frames

# Launch RQT node graph
ros2 run rqt_graph rqt_graph

# Test Dijkstra standalone (before ROS integration)
cd ~/srm_buggy_ws/src/buggy_brain/buggy_brain
python3 map_graph.py
```

---

## 📅 Day-by-Day Schedule

| Day | Team | Deliverable | Success Criterion |
|----|------|------------|-------------------|
| **1** | Alpha | Base vehicle + workspace | Buggy moves on `/cmd_vel` pub |
| **2** | Alpha | All 5 sensor plugins + campus world | All 8 sensor topics at correct Hz |
| **3** | Bravo | Dijkstra + path planner + waypoint follower | Buggy moves toward typed destination |
| **4** | Charlie | Obstacle detector + state machine + ultrasonics | EMERGENCY_STOP on `/obstacle_detected: true` |
| **5** | All | Full system integration | End-to-end run to B and C |
| **6** | All | Obstacle avoidance + crowd handoff in-loop | Stop-resume + crowd countdown working |
| **7** | Charlie | RViz2 polish + `run_demo.sh` | One-command demo under 5 minutes |
| **8** | All | Stress test — 10 consecutive runs | ≥ 8/10 runs pass |
| **9** | Bravo | RQT graph + docs + GitHub push | Final repo clean, README done |
| **10** | All | HOD demo rehearsal + Q&A prep | Live demo + traceability print ready |

---

## ⚠️ Known Pitfalls & Fixes

| Problem | Symptom | Fix |
|---------|---------|-----|
| **Clock desync** | TF errors, sensor data missed | Add `use_sim_time:=True` to **every** node in launch file |
| **Ray sensor no topic** | `/scan` absent from `topic list` | Check `<remapping>~/out:=scan</remapping>` in plugin config |
| **Waypoint spinning** | Buggy rotates endlessly | Reduce angular P-gain `1.2 → 0.7`, add deadband < `0.05 rad` |
| **Waypoint oscillation** | Buggy bounces near waypoint | Increase arrival radius `0.8 → 1.2 m`, add min linear speed `0.3 m/s` |
| **`input()` freezes ROS** | Node hangs at keyboard prompt | Wrap `input()` in `threading.Thread(daemon=True)` |
| **Gazebo crashes on 3rd run** | `gzserver` killed | Add `pkill -f gzserver && pkill -f gzclient && sleep 2` to `run_demo.sh` |
| **Odometry drift** | Buggy arrives at wrong coords | ✅ Document it. Say *"GPS fusion corrects drift in Phase 2"* |
| **colcon build fails** | Missing package errors | Read error carefully → `sudo apt install ros-humble-[pkg]` |
| **URDF spawn invisible** | Robot missing from Gazebo | Confirm `spawn_entity.py` receives correct URDF from `robot_state_publisher` |
| **LaserScan all zeros** | `/scan` ranges all `0.0` | Add wall models to world file — Gazebo needs surfaces to return non-inf |

---

## 🔭 Phase 2 Roadmap

| Phase 1 (This Repo — SITL) | Phase 2 (Hardware) | Change Needed |
|---------------------------|-------------------|--------------|
| Odometry-only localisation | GPS + IMU EKF fusion | `robot_localization` package |
| Proportional waypoint follower | Nav2 DWB controller | Costmap + DWB local planner |
| Hardcoded campus graph | SLAM-built dynamic map | `slam_toolbox` |
| LiDAR ray-count crowd proxy | YOLOv8 on RealSense | Train pedestrian model |
| Simulated Gazebo sensors | Physical sensor hardware | ROS 2 hardware driver nodes |
| Terminal input | Tablet / Web UI | REST API or `rosbridge` |
| Differential drive URDF | Physical motor CAN bus | CAN bus / PWM motor driver node |

> **What transfers unchanged from Phase 1 → Phase 2:**  
> `state_machine.py` · `obstacle_detector.py` threshold logic · `map_graph.py` Dijkstra · all ROS 2 topic names and message types · launch file structure

---

## ❓ Q&A Quick Reference

| Question | Answer |
|----------|--------|
| *Why Dijkstra and not A\*?* | Dijkstra guarantees optimal path on our 4-node static graph. A\* heuristic only helps on large grids. Phase 2 uses Nav2 global planner. |
| *Why not Nav2?* | Nav2 requires dynamic costmaps, SLAM, and AMCL — Phase 2 work. Phase 1 proves the sensor-to-action pipeline works. |
| *Is crowd detection real?* | Phase 1 uses LiDAR ray density (physically meaningful). Phase 2 replaces this with YOLOv8 on RealSense D435i. |
| *Why Python and not C++?* | Python ROS 2 nodes are architecturally identical. At 10 Hz simulation rate, Python is sufficient. Phase 2 perception nodes will be C++. |
| *Will this run on the real buggy?* | Yes. State machine, obstacle detector, and Dijkstra transfer unchanged. Gazebo sensor topics are replaced by physical driver topics with the same message types. |

---

## ✅ Final Delivery Checklist

```
MINIMUM VIABLE DEMO — Phase 1 is a success if ALL 5 are true:
  ✓  User types destination  →  path computes and prints
  ✓  Buggy moves autonomously toward destination
  ✓  Buggy stops at obstacle and resumes when clear
  ✓  Buggy announces arrival at destination
  ✓  System can be relaunched and run again immediately
```

| Item | Status |
|------|--------|
| `colcon build` passes with zero errors | `[x]` |
| All 5 sensor topics publishing at correct Hz | `[x]` |
| Gazebo campus world renders with road network | `[x]` |
| Dijkstra computes correct path for all 3 destinations | `[x]` |
| Buggy navigates START → B autonomously | `[x]` |
| Buggy navigates START → C autonomously | `[x]` |
| Emergency stop triggers at 1.5 m | `[x]` |
| Buggy resumes after 5 clear LiDAR readings | `[x]` |
| Ultrasonic alert triggers at 0.30 m | `[ ]` |
| Crowd detection triggers with 40+ LiDAR rays | `[ ]` |
| MANUAL_CONTROL entered after countdown | `[ ]` |
| Arrival announcement printed in terminal | `[x]` |
| System accepts new destination after arrival | `[x]` |
| RViz2 shows: model, pointcloud, camera, markers | `[ ]` |
| `run_demo.sh` launches everything in one command | `[x]` |
| 8/10 stress test runs pass | `[ ]` |
| GitHub repository pushed with README + traceability | `[x]` |
| RQT node graph screenshot saved to `docs/` | `[ ]` |

---

## 🤝 Contributing Guidelines

> **Read the full [CONTRIBUTING.md](CONTRIBUTING.md) before making any changes to this repository.**

A quick summary of the rules every team member must follow:

### Branching

Always branch off `main`. Never commit directly to `main`.

```bash
git checkout main
git pull origin main
git checkout -b feature/your-feature-name
```

| Prefix | Use for | Example |
|--------|---------|---------|
| `feature/` | New node, behaviour, or sensor | `feature/crowd-detector` |
| `fix/` | Bug fix in existing code | `fix/waypoint-oscillation` |
| `docs/` | README, comments, diagrams only | `docs/update-traceability` |
| `test/` | Adding or fixing test scripts | `test/stress-run-script` |

### Commits

Write short, clear commit messages with a prefix so teammates can see what changed at a glance.

```bash
# Good
git commit -m "feat: add ultrasonic monitor node with 0.30 m threshold"
git commit -m "fix: increase arrival radius to stop waypoint oscillation"
git commit -m "docs: add Day 5 integration notes to README"

# Bad
git commit -m "changes"
git commit -m "fix stuff"
```

### Pull Requests

Every change goes through a Pull Request. No direct pushes to `main`, no exceptions.

- Assign at least one teammate as reviewer before requesting a merge
- The PR description must state what changed, why it changed, and how you tested it
- All review comments must be resolved before merging

### Hard Rules

- **No force pushes** — `git push --force` is banned on all branches
- **No rebasing shared branches** — rebase only your own local branch before its first push
- **Always merge with `--no-ff`** — keep the history clean and traceable
- Delete your feature branch after it is merged

### Code Standards

- All Python nodes must pass `colcon build` with zero errors before a PR is opened
- Every new threshold or state transition must have a comment referencing the plan section it comes from (e.g. `# §5.1 — LiDAR stop threshold`)
- Do not change any existing ROS 2 topic names or message types without team discussion — these are shared contracts between all nodes
- Only `speed_controller.py` is allowed to publish to `/cmd_vel`

For the complete workflow with full examples, see [CONTRIBUTING.md](CONTRIBUTING.md).

---

## 👥 Team

**SRM Institute of Science and Technology**  
Department of Artificial Intelligence and Machine Learning.  
6-Person Autonomous Systems Team · Phase 1 SITL Sprint · March 2026

| Sub-Team | Members | Responsibility |
|----------|---------|---------------|
| **Team Alpha** | Member 1 + Member 2 | URDF model · sensor plugins · campus world · launch files |
| **Team Bravo** | Member 3 + Member 4 | Dijkstra graph · path planner · waypoint follower · state machine |
| **Team Charlie** | Member 5 + Member 6 | Obstacle detection · ultrasonics · crowd detector · RViz2 visualizer · demo scripts |

---

<div align="center">

**STRICTLY CONFIDENTIAL — INTERNAL USE ONLY**

*SRM Institute of Science and Technology | Autonomous Electric Buggy | Phase 1 SITL | v1.0 | March 2026*

```
"Build the software first. Let the hardware catch up."
— SRM Autonomous Buggy Team
```

</div>
