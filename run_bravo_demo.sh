#!/bin/bash
# ============================================================
#  run_bravo_demo.sh  —  SRM Autonomous Buggy SITL Demo
#  Works standalone WITHOUT Team Charlie sensor nodes.
#
#  Usage:
#    cd ~/Documents/projects/srm_buggy_ws
#    ./run_bravo_demo.sh
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================"
echo "  SRM Autonomous Buggy — SITL Demo"
echo "  Standalone (sensor stubs active)"
echo "======================================"

# Source workspace
source install/setup.bash

# ── Cleanup old processes ─────────────────────────────────
echo "[1] Stopping any previous processes..."
killall -9 gzserver gzclient 2>/dev/null || true
pkill -9 -f gazebo          2>/dev/null || true
pkill -9 -f "state_machine"    2>/dev/null || true
pkill -9 -f "speed_controller" 2>/dev/null || true
pkill -9 -f "waypoint_follower" 2>/dev/null || true
pkill -9 -f "path_planner"     2>/dev/null || true
pkill -9 -f "obstacle_detector" 2>/dev/null || true
pkill -9 -f "sensor_stub"      2>/dev/null || true
sleep 5

# ── Launch Gazebo ─────────────────────────────────────────
echo "[2] Launching Gazebo + campus world..."
ros2 launch buggy_bringup full_system.launch.py &
GAZEBO_PID=$!
echo "    Waiting 20 s for Gazebo to fully start..."
sleep 20

# Verify Gazebo started
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "ERROR: Gazebo failed to start! Check GL drivers."
    exit 1
fi
echo "    Gazebo running (PID=$GAZEBO_PID)"

# ── Start sensor stubs (replaces Team Charlie) ────────────
echo "[3] Starting sensor stub (replace Team Charlie)..."
ros2 run buggy_brain sensor_stub &
STUB_PID=$!
sleep 1
echo "    sensor_stub running (PID=$STUB_PID)"

# ── Start brain nodes ─────────────────────────────────────
echo "[4] Starting state_machine..."
ros2 run buggy_brain state_machine &
SM_PID=$!
sleep 2

echo "[5] Starting speed_controller..."
ros2 run buggy_brain speed_controller &
SC_PID=$!
sleep 1

echo "[6] Starting waypoint_follower..."
ros2 run buggy_brain waypoint_follower &
WF_PID=$!
sleep 1

echo "[7] Starting obstacle_detector..."
ros2 run buggy_brain obstacle_detector &
OD_PID=$!
sleep 2

# ── Interactive path planner ──────────────────────────────
echo ""
echo "[8] Starting path_planner — select destination below:"
echo "    (A) SRM Institute (North)"
echo "    (B) SRM Hospital  (East)"
echo "    (C) SRM Temple    (South)"
echo "    (D) Buggy Hub     (return home)"
echo ""
ros2 run buggy_brain path_planner

# ── Cleanup on exit ───────────────────────────────────────
echo ""
echo "Demo ended. Cleaning up..."
kill $SM_PID $SC_PID $WF_PID $OD_PID $STUB_PID $GAZEBO_PID 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
