#!/bin/bash
# Wrapper script to run Isaac Sim with ROS2 properly configured
# This script ISOLATES Isaac Sim's Python 3.11 + internal ROS2 Jazzy environment

set -e  # Exit on error

# ============================================================================
# STEP 1: UNSET ALL SYSTEM ROS2 VARIABLES TO PREVENT CONFLICTS
# ============================================================================
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Cleaning System ROS2 Environment                        ║"
echo "╚═══════════════════════════════════════════════════════════╝"

# Unset all ROS2 environment variables from system installation
unset ROS_VERSION
unset ROS_PYTHON_VERSION
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# CRITICAL: Completely unset PYTHONPATH to prevent system ROS2 contamination
# Isaac Sim will rebuild its own PYTHONPATH internally
unset PYTHONPATH

# Clean LD_LIBRARY_PATH of system ROS2 paths
if [ -n "$LD_LIBRARY_PATH" ]; then
    LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v -E '/(ros|jazzy|humble|iron)/' | tr '\n' ':' | sed 's/:$//')
fi

echo "✓ Cleaned system ROS2 variables"
echo ""

# ============================================================================
# STEP 2: SET UP ISAAC SIM'S INTERNAL ROS2 ENVIRONMENT
# ============================================================================
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Configuring Isaac Sim's Internal ROS2 (Jazzy)          ║"
echo "╚═══════════════════════════════════════════════════════════╝"

# Isaac Sim paths
ISAAC_SIM_ROOT="$HOME/isaacsim_5_1_pegasus"
ISAAC_ROS2_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/jazzy"

# Verify Isaac Sim installation
if [ ! -d "$ISAAC_SIM_ROOT" ]; then
    echo "❌ ERROR: Isaac Sim not found at $ISAAC_SIM_ROOT"
    echo "Please install Isaac Sim 5.1 first"
    exit 1
fi

if [ ! -d "$ISAAC_ROS2_PATH" ]; then
    echo "❌ ERROR: Isaac Sim's internal ROS2 not found at $ISAAC_ROS2_PATH"
    echo "Make sure the isaacsim.ros2.bridge extension is installed"
    exit 1
fi

# Set ONLY Isaac Sim's ROS2 environment
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add Isaac Sim's ROS2 libraries (PREPEND to take priority)
export LD_LIBRARY_PATH="$ISAAC_ROS2_PATH/lib:$LD_LIBRARY_PATH"

# Set PYTHONPATH to ONLY Isaac Sim's internal ROS2 (no system paths!)
export PYTHONPATH="$ISAAC_ROS2_PATH:$ISAAC_ROS2_PATH/rclpy"

echo "✓ ROS_DISTRO: $ROS_DISTRO"
echo "✓ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "✓ Using Isaac Sim's internal ROS2: $ISAAC_ROS2_PATH"
echo "✓ Python environment: Isaac Sim Python 3.11"
echo ""

# ============================================================================
# STEP 3: VERIFY ENVIRONMENT
# ============================================================================
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Environment Verification                                ║"
echo "╚═══════════════════════════════════════════════════════════╝"

# Check if rclpy exists in Isaac Sim's ROS2
if [ -d "$ISAAC_ROS2_PATH/rclpy" ]; then
    echo "✓ Found Isaac Sim's rclpy at: $ISAAC_ROS2_PATH/rclpy"
else
    echo "⚠ WARNING: rclpy directory not found in expected location"
fi

# Check for required libraries
if [ -f "$ISAAC_ROS2_PATH/lib/librclcpp.so" ]; then
    echo "✓ Found ROS2 core libraries"
else
    echo "⚠ WARNING: ROS2 core libraries not found"
fi

echo ""

# ============================================================================
# STEP 4: LAUNCH ISAAC SIM WITH ISOLATED ENVIRONMENT
# ============================================================================
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Starting Isaac Sim...                                   ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Script: 1_px4_single_vehicle_copy.py"
echo "This will create ROS2 topics:"
echo "  - /isaac_sim/reset_request"
echo "  - /isaac_sim/reset_done"
echo ""

# Change to script directory
cd "$(dirname "$0")"

# Run using Isaac Sim's Python wrapper (Python 3.11)
# Use env -i to start with a completely clean environment, then add only what we need
env -i \
    HOME="$HOME" \
    USER="$USER" \
    PATH="$PATH" \
    DISPLAY="$DISPLAY" \
    TERM="$TERM" \
    ROS_DISTRO="jazzy" \
    RMW_IMPLEMENTATION="rmw_cyclonedds_cpp" \
    LD_LIBRARY_PATH="$ISAAC_ROS2_PATH/lib" \
    "$ISAAC_SIM_ROOT/python.sh" 1_px4_single_vehicle_copy.py
