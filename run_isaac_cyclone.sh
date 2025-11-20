#!/bin/bash
# Isaac Sim startup script configured for CycloneDDS (to match C++ trainer)
# Uses Isaac Sim's INTERNAL ROS2 to avoid Python version conflicts

set -e

# Isaac Sim paths
ISAAC_SIM_ROOT="$HOME/isaacsim_5_1_pegasus"
ISAAC_ROS2_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/jazzy"

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Starting Isaac Sim with CycloneDDS                      ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "ROS_DISTRO: jazzy"
echo "RMW_IMPLEMENTATION: rmw_cyclonedds_cpp"
echo "ROS_DOMAIN_ID: 0"
echo "Using Isaac Sim's internal ROS2 (Python 3.11 compatible)"
echo ""

# Verify Isaac Sim installation
if [ ! -d "$ISAAC_SIM_ROOT" ]; then
    echo "❌ ERROR: Isaac Sim not found at $ISAAC_SIM_ROOT"
    exit 1
fi

# Change to script directory
cd "$(dirname "$0")"

# Run using env -i to prevent system ROS2 contamination
# Isaac Sim's python.sh will rebuild PYTHONPATH internally
env -i \
    HOME="$HOME" \
    USER="$USER" \
    PATH="$PATH" \
    DISPLAY="$DISPLAY" \
    TERM="$TERM" \
    ROS_DISTRO="jazzy" \
    ROS_DOMAIN_ID="0" \
    RMW_IMPLEMENTATION="rmw_cyclonedds_cpp" \
    LD_LIBRARY_PATH="$ISAAC_ROS2_PATH/lib" \
    "$ISAAC_SIM_ROOT/python.sh" 1_px4_single_vehicle_copy.py
