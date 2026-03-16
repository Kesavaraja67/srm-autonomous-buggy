# Team Alpha — Complete Instructions
## SRM Autonomous Buggy · Phase 1 SITL · March 2026

---

## Your Role

**Team Alpha owns:** `buggy_description/` (URDF + meshes) and `buggy_bringup/` (launch files + world file + RViz config)

**You do NOT write any Python nodes.** Your deliverable is the virtual vehicle, the virtual world, and the launch system that lets everyone else's nodes run.

**Big news:** Your URDF (`buggy_urdf.xacro`) is already complete with all 8 sensor links and all Gazebo plugins. This means Days 1–2 are mostly setup and verification, not writing. Use that time advantage to build your world file and launch files carefully.

---

## File Placement Map

After you finish all your work, your contributions live here:

```text
srm_buggy_ws/src/
├── buggy_description/
│   └── urdf/
│       └── buggy_urdf.xacro          ← ALREADY DONE (your uploaded file)
│
└── buggy_bringup/
    ├── launch/
    │   ├── campus_world.launch.py    ← You create this (Day 2)
    │   ├── buggy_spawn.launch.py     ← You create this (Day 2)
    │   └── full_system.launch.py     ← You create this (Day 5 integration)
    ├── worlds/
    │   └── srm_campus.world          ← You create this (Day 2) — the SDF environment
    └── rviz/
        └── srm_buggy_demo.rviz       ← Team Charlie creates (Day 7), you review
```

---

## Day 1 — Environment Setup + Base Vehicle

**Your goal:** `colcon build` passes. Buggy spawns in empty Gazebo. `/cmd_vel` and `/odom` topics exist.

### Step 1 — Git setup (do this first, before any files)

```bash
# Clone the team repo
cd ~/srm_buggy_ws/src
git clone https://github.com/<your-org>/srm-autonomous-buggy.git .

# Always start from fresh main
git checkout main
git pull origin main

# Create your Day 1 branch
git checkout -b feature/alpha-day1-workspace-setup
```

**Branch name:** `feature/alpha-day1-workspace-setup`

### Step 2 — Install all dependencies

```bash
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

### Step 3 — Source ROS permanently

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4 — Create workspace structure

```bash
mkdir -p ~/srm_buggy_ws/src
cd ~/srm_buggy_ws
```

### Step 5 — Create the buggy_description package

```bash
cd ~/srm_buggy_ws/src
ros2 pkg create --build-type ament_cmake buggy_description
mkdir -p buggy_description/urdf buggy_description/meshes buggy_description/launch
```

Copy your XACRO file into the urdf folder:
```bash
cp /path/to/buggy_urdf.xacro ~/srm_buggy_ws/src/buggy_description/urdf/buggy_urdf.xacro
```

Edit `buggy_description/CMakeLists.txt` — add this before `ament_package()`:
```cmake
install(DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)
```

Edit `buggy_description/package.xml` — add these dependencies:
```xml
<depend>urdf</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
<depend>joint_state_publisher</depend>
<depend>gazebo_ros</depend>
```

### Step 6 — Create the buggy_bringup package

```bash
cd ~/srm_buggy_ws/src
ros2 pkg create --build-type ament_cmake buggy_bringup
mkdir -p buggy_bringup/launch buggy_bringup/worlds buggy_bringup/rviz buggy_bringup/include buggy_bringup/src
```

Edit `buggy_bringup/CMakeLists.txt` — add this before `ament_package()`:
```cmake
install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)
```

Edit `buggy_bringup/package.xml` — add:
```xml
<depend>gazebo_ros</depend>
<depend>robot_state_publisher</depend>
<depend>xacro</depend>
<exec_depend>ros2launch</exec_depend>
```

### Step 7 — Place your launch files

Copy the 3 launch files you received into:
```bash
cp campus_world.launch.py   ~/srm_buggy_ws/src/buggy_bringup/launch/
cp buggy_spawn.launch.py    ~/srm_buggy_ws/src/buggy_bringup/launch/
cp full_system.launch.py    ~/srm_buggy_ws/src/buggy_bringup/launch/
```

### Step 8 — Set up .gitignore (if not already done)

```bash
cat > ~/srm_buggy_ws/.gitignore << 'EOF'
build/
install/
log/
__pycache__/
*.pyc
*.pyo
.vscode/
*.swp
EOF
```

### Step 9 — Build and verify

```bash
cd ~/srm_buggy_ws
colcon build --symlink-install
source install/setup.bash
```

**Expected output:** `Summary: X packages finished [...]` with zero errors.

### Step 10 — Quick vehicle spawn test (empty world)

```bash
# Terminal 1: launch Gazebo empty world
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: start robot_state_publisher
cd ~/srm_buggy_ws
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/buggy_description/urdf/buggy_urdf.xacro)"

# Terminal 3: spawn the buggy
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description -entity srm_aquila_buggy -x -20 -y 0 -z 0.425
```

### Step 11 — Verify Day 1 success criterion

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once
```
**The buggy must move forward. If it does not, Day 1 has failed.**

Also verify:
```bash
ros2 topic list | grep -E "cmd_vel|odom"
# Must show both /cmd_vel and /odom
```

### Step 12 — Commit and push

```bash
cd ~/srm_buggy_ws
git add src/buggy_description/ src/buggy_bringup/ .gitignore
git commit -m "feat: add buggy_description and buggy_bringup package scaffolding (§2.4)"
git push origin feature/alpha-day1-workspace-setup
```

Then open a PR on GitHub titled:
`feat: initial workspace setup, URDF packages, and base vehicle spawn`

Assign your Team Alpha partner as reviewer.

---

## Day 2 — Sensor Plugins + Campus World

**Your goal:** All 8 sensor topics publishing at correct Hz. Campus world renders with road network.

### Step 1 — Create your Day 2 branch

```bash
git checkout main
git pull origin main
git checkout -b feature/alpha-day2-sensors-world
```

**Branch name:** `feature/alpha-day2-sensors-world`

### Step 2 — Place your world file

Your `srm_campus.world` (provided in the deliverables package) goes here:
```bash
cp srm_campus.world ~/srm_buggy_ws/src/buggy_bringup/worlds/
```

The world file already contains:
- Grey ground plane
- 3 road segments (START→HUB, HUB→Library B, HUB→Admin C) matching §2.2 exactly
- White kerb lines on all roads
- Building walls (0.3m thick, 3m tall) giving LiDAR return surfaces
- Coloured destination markers (Blue=START, Green=B, Orange=C)
- 2 obstacle boxes at (-10,0) and (10,0)
- 6 crowd cylinders in a cluster at ~(5,0)

### Step 3 — Rebuild

```bash
cd ~/srm_buggy_ws
colcon build --symlink-install
source install/setup.bash
```

### Step 4 — Launch with campus world

```bash
# Full spawn test with campus world
ros2 launch buggy_bringup campus_world.launch.py
# In another terminal:
ros2 launch buggy_bringup buggy_spawn.launch.py
```

### Step 5 — Verify all 8 sensor topics

```bash
ros2 topic list | grep -E "scan|camera|imu|gps|ultrasonic|odom"
```

**Expected — must see ALL of these:**
```text
/camera/image_raw
/gps/fix
/imu/data
/odom
/scan
/ultrasonic/front
/ultrasonic/left
/ultrasonic/rear
/ultrasonic/right
```

If `/scan` is missing, the LiDAR plugin remapping is broken.
Fix: check the `<remapping>~/out:=scan</remapping>` line in the XACRO file.

If `/ultrasonic/front` is missing, same issue — check the remapping for each ultrasonic plugin.

### Step 6 — Verify LiDAR rate

```bash
ros2 topic hz /scan
# Expected: ~10.0 Hz
```

### Step 7 — Verify camera

```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw
# Must show a live image of the campus world (grey road, coloured markers)
```

### Step 8 — Verify LiDAR is not all zeros

```bash
ros2 topic echo /scan --once | head -30
# ranges[] must show real float values — NOT all 0.0 or all inf
```

If all zeros: the world has no objects in range. Your world file walls should fix this.
If still zeros: in Gazebo, click Insert → Simple Shapes → Box and place a box in front of the buggy.

### Step 9 — Verify TF tree

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf — must show: odom → base_footprint → base_link → lidar_link, camera_link, etc.
```

### Step 10 — Commit and push

```bash
git add src/buggy_bringup/worlds/srm_campus.world \
        src/buggy_bringup/launch/campus_world.launch.py \
        src/buggy_bringup/launch/buggy_spawn.launch.py
git commit -m "feat: add srm_campus.world with roads/walls/markers and sensor launch files (§2.2, §3.2, §4.1)"
git push origin feature/alpha-day2-sensors-world
```

PR title: `feat: campus world SDF with road network and all sensor topics at correct Hz`

---

## Day 3-4 — Support + Review

Team Bravo and Charlie are writing Python nodes. Your job:

1. **Review their PRs** — you are secondary reviewer for `buggy_brain/` files. Check they don't change topic names.
2. **Answer questions** about sensor topics — you own `/scan`, `/ultrasonic/*`, `/camera/*`, `/odom`. If their nodes can't subscribe, help them debug topic names.
3. **Test that your world is usable** — place obstacle_box_1 in front of the buggy and run:
   ```bash
   ros2 topic echo /scan | grep -v inf | head -5
   # Should show range values less than 1.5 m when box is close
   ```
4. **Create a placeholder RViz config** so Team Charlie has something to edit on Day 7:
   ```bash
   ros2 launch buggy_bringup buggy_spawn.launch.py
   rviz2
   # Add displays: RobotModel, LaserScan (topic=/scan), TF
   # Save config to: src/buggy_bringup/rviz/srm_buggy_demo.rviz
   # File → Save Config As → srm_buggy_demo.rviz
   ```

   Commit this on branch `feature/alpha-placeholder-rviz`:
   ```bash
   git commit -m "feat: add placeholder RViz2 config with robot model and LiDAR display"
   ```

---

## Day 5 — Full System Integration

**This is the most important day.** All three teams merge their work and run the first end-to-end demo.

### Step 1 — Create integration branch

```bash
git checkout main
git pull origin main
git checkout -b feature/alpha-day5-integration
```

### Step 2 — Place full_system.launch.py

Your `full_system.launch.py` is already written and includes all brain nodes.
It should already be in `buggy_bringup/launch/`.

### Step 3 — Build and launch

```bash
cd ~/srm_buggy_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch buggy_bringup full_system.launch.py
```

### Step 4 — Common integration problems and fixes

**Problem: TF errors, sensor data missing**
```bash
# Cause: some node not using sim time
# Fix: confirm use_sim_time:=true on EVERY node in full_system.launch.py
ros2 param get /obstacle_detector_node use_sim_time
# Must return: True
```

**Problem: Buggy spins in circles**
```bash
# Cause: angular P-gain too high
# Fix: Team Bravo reduces angular gain in waypoint_follower.py from 1.2 → 0.7
# You just need to tell them
```

**Problem: Buggy doesn't stop before obstacle**
```bash
# Check that /scan is publishing and obstacle_detector is running
ros2 topic echo /obstacle_detected
# Should show {data: true} when box is within 1.5 m
```

**Problem: `/cmd_vel` no output**
```bash
# Check state machine state
ros2 topic echo /buggy_state
# Should be NAVIGATING when a destination is selected
```

### Step 5 — Run end-to-end test

In the full_system terminal, type `B` when prompted.
Watch Gazebo — buggy should start moving from (-20, 0) toward (0, 0) then toward (20, 0).

Record the run on screen (use `recordmydesktop` or `obs-studio`).

### Step 6 — Commit

```bash
git add src/buggy_bringup/launch/full_system.launch.py
git commit -m "feat: full_system.launch.py integrates all teams — first end-to-end run (§Day 5)"
git push origin feature/alpha-day5-integration
```

---

## Day 6 — Obstacle Testing Support

Your role today:

1. In Gazebo, drag obstacle_box_1 onto road segment START→HUB using the GUI
2. Confirm buggy stops when LiDAR sees the box at < 1.5 m
3. Drag obstacle off road → confirm buggy resumes after 5 clear readings
4. Move crowd cylinder cluster to be in front of buggy path (around x=5, y=0)
5. Confirm crowd detection triggers (40+ ray threshold)

**Nothing to commit unless you find a bug in your world file.**

If you find that the walls aren't giving LiDAR returns:
```bash
# Check raw scan for zeros near walls
ros2 topic echo /scan --once
# If ranges all inf, walls may need to be thicker or closer
# Fix: increase wall thickness from 0.3m to 0.5m in world file
# Commit on: fix/alpha-world-wall-lidar-returns
```

---

## Day 7 — RViz Polish

**Branch name:** `feature/alpha-rviz-polish`

Team Charlie owns the RViz config but you need to help.

1. Launch the full system
2. In RViz, add/verify these displays:
   - **RobotModel** — Fixed Frame: `odom`
   - **LaserScan** — topic: `/scan`, color: Rainbow, size: 0.05
   - **Camera** — topic: `/camera/image_raw`
   - **Path** — topic: `/planned_path`, line color: yellow
   - **MarkerArray** — topic: `/visualization_markers`
3. Set Fixed Frame to `odom`
4. Save config to `buggy_bringup/rviz/srm_buggy_demo.rviz`

```bash
git add src/buggy_bringup/rviz/srm_buggy_demo.rviz
git commit -m "feat: save RViz2 config with robot model, LiDAR, camera, path, markers (§Day 7)"
```

---

## Day 8 — Stress Test Support

Run `./run_demo.sh` 10 times. Record pass/fail:

| Run | Launch OK | Dest input | Moves | Obstacle stop | Resume | Arrived |
|-----|-----------|-----------|-------|--------------|--------|---------|
| 1   | [ ]       | [ ]       | [ ]   | [ ]          | [ ]    | [ ]     |
| 2   | [ ]       | [ ]       | [ ]   | [ ]          | [ ]    | [ ]     |
| ... |           |           |       |              |        |         |

**Common Day 8 failures from your side:**

If Gazebo crashes on 3rd run, that's a `run_demo.sh` problem (Team Charlie owns it), but tell them to add:
```bash
pkill -f gzserver && pkill -f gzclient && sleep 2
```

If spawn fails on restart, your spawn might not be waiting long enough. The `TimerAction(period=3.0)` in `full_system.launch.py` handles this. Increase to 5.0 if needed.

---

## Day 9 — Documentation

**Branch name:** `docs/alpha-readme-sensor-table`

Add or verify the sensor table in README.md is correct (matches your XACRO positions).
Generate TF tree screenshot:
```bash
ros2 run tf2_tools view_frames
# Saves frames.pdf — convert to PNG
cp frames.pdf ~/srm_buggy_ws/src/docs/assets/tf_tree.pdf
```

```bash
git commit -m "docs: add TF tree diagram and verify sensor position table in README"
```

---

## Day 10 — HOD Demo

Your demo talking points:

1. **"This is the virtual vehicle"** — Point at RViz RobotModel — explain it is based on the actual Aquila EV dimensions (4.12 m × 1.2 m × 1.9 m)
2. **"These are the physical sensors"** — Point at LiDAR (roof, 1.85 m), Camera (front, 1.2 m), 4 ultrasonics (corners, 0.3 m)
3. **"This is the campus environment"** — Point at Gazebo world — explain road coordinates match the Dijkstra graph exactly (§2.2)
4. **"The world provides realistic sensor data"** — Explain why walls matter for LiDAR (surfaces must exist to return ranges)

---

## Quick Verification Checklist

Run this before every PR:

```bash
# 1. Build passes
cd ~/srm_buggy_ws && colcon build --symlink-install
# Expected: zero errors

# 2. Source workspace
source install/setup.bash

# 3. All sensor topics alive (after spawning buggy)
ros2 topic list | grep -E "scan|camera|imu|gps|ultrasonic|odom"
# Must show all 8

# 4. LiDAR at correct rate
ros2 topic hz /scan
# Expected: ~10 Hz

# 5. Buggy moves on cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once
# Buggy must move in Gazebo

# 6. LiDAR returns are real values (not zeros/inf)
ros2 topic echo /scan --once | grep "ranges" | head -3
# Must show float values like [2.3, 2.3, 2.2, ...]
```

---

## Git Quick Reference for Team Alpha

| Situation | Branch Name |
|-----------|-------------|
| Day 1 workspace setup | `feature/alpha-day1-workspace-setup` |
| Day 2 sensors + world | `feature/alpha-day2-sensors-world` |
| RViz placeholder | `feature/alpha-placeholder-rviz` |
| Day 5 integration | `feature/alpha-day5-integration` |
| Fix wall LiDAR returns | `fix/alpha-world-wall-lidar-returns` |
| Fix spawn delay | `fix/alpha-spawn-gazebo-timing` |
| RViz polish | `feature/alpha-rviz-polish` |
| Docs update | `docs/alpha-readme-sensor-table` |

**Golden rules — never break these:**
- Never commit directly to `main`
- Never force push (`git push --force` is banned)
- Every PR needs at least one reviewer (cross-team preferred)
- `colcon build` must pass before opening any PR

---

## Frequently Asked Questions

**Q: What format is the world file? URDF or SDF?**
The world file (`srm_campus.world`) is **SDF** format, not URDF. URDF is for robot descriptions. SDF is for Gazebo worlds — static buildings, roads, obstacles. Gazebo Classic reads SDF natively. Your robot is spawned separately using `spawn_entity.py` which reads from the URDF via the `/robot_description` topic.

**Q: Why does my URDF have sensors but I still need a world file?**
The URDF defines the robot (sensors, wheels, joints). The world file defines the environment (roads, buildings, obstacles). Sensors only work if there are surfaces in the world to return signals — LiDAR returns `inf` if there are no walls or ground features in range.

**Q: What if `colcon build` says a package is missing?**
Read the exact error message. It will say something like `Could not find package 'gazebo_ros'`. Run `sudo apt install ros-humble-gazebo-ros-pkgs` and rebuild.

**Q: The buggy spawns but falls through the ground — what's wrong?**
The `chassis_z` property in your XACRO is `0.425` (wheel_radius + chassis_height/2 + 0.04 = 0.26 + 0.125 + 0.04). Spawn with `-z 0.425` to match. If you spawn at `z=0`, the wheels clip through the ground on the first physics tick.

**Q: How do I move the obstacle boxes in Gazebo to test obstacle detection?**
In Gazebo, click **Edit** (top menu) → **Translation Mode**, then click an obstacle box to select it, then drag it onto a road in front of the buggy. You can also drag it off the road to test the resume behaviour.

---

*SRM Institute of Science and Technology · Team Alpha · SITL Sprint · March 2026*
