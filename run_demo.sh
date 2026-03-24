#!/bin/bash
# =============================================================================
# SRM Autonomous Electric Campus Buggy — Demo Launch Script
# Team Charlie owns this file.
# Usage: bash run_demo.sh [--headless] [--world <world_name>]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# Default options
WORLD="srm_campus"
HEADLESS=false

# Parse args
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --headless) HEADLESS=true ;;
        --world) WORLD="$2"; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

echo "========================================"
echo " SRM Autonomous Buggy — Demo Runner"
echo " World : $WORLD"
echo " Headless: $HEADLESS"
echo "========================================"

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace install (after build)
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[WARN] Workspace not built yet. Run 'colcon build' first."
    echo "       cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi

# Launch Gazebo + RViz + all nodes
if [ "$HEADLESS" = true ]; then
    export HEADLESS=1
    ros2 launch buggy_bringup full_system.launch.py world:="$WORLD" headless:=true
else
    ros2 launch buggy_bringup full_system.launch.py world:="$WORLD" headless:=false
fi
