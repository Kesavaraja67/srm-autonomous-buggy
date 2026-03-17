#!/bin/bash
echo "============================================="
echo "Team Bravo Integrity Check"
echo "============================================="

# 1. Build workspace
echo "[1] Building the workspace..."
colcon build --symlink-install
if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi
echo "✅ Build passed with zero errors."

# 2. Source the workspace
source install/setup.bash

# 3. Test Dijkstra Graph
echo ""
echo "[2] Running Map Graph Dijkstra stand-alone test..."
python3 src/buggy_brain/buggy_brain/map_graph.py

# 4. Check for ROS 2 launch capabilities of the new nodes
echo ""
echo "[3] Checking if new nodes are registered as executables..."
if ros2 pkg executables buggy_brain | grep -q "speed_controller"; then
    echo "✅ speed_controller registered successfully."
else
    echo "❌ speed_controller missing from setup.py entry_points!"
fi

if ros2 pkg executables buggy_brain | grep -q "path_planner"; then
    echo "✅ path_planner registered successfully."
else
    echo "❌ path_planner missing from setup.py entry_points!"
fi

echo ""
echo "[4] Final check complete. To run full end-to-end integration tests, open 3 terminals:"
echo "Terminal 1: ros2 run buggy_brain state_machine"
echo "Terminal 2: ros2 run buggy_brain speed_controller"
echo "Terminal 3: ros2 topic pub /obstacle_detected std_msgs/msg/Bool '{data: true}' -1"
echo "Terminal 4: ros2 topic echo /cmd_vel"
echo "And verify that cmd_vel drops to 0.0"
