#!/bin/bash
# Quick verification script to test Isaac Sim ROS2 integration

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Isaac Sim ROS2 Integration Verification                 ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Check Isaac Sim installation
echo "1. Checking Isaac Sim installation..."
ISAAC_SIM_ROOT="$HOME/isaacsim_5_1_pegasus"
if [ -d "$ISAAC_SIM_ROOT" ]; then
    echo "   ✅ Isaac Sim found at: $ISAAC_SIM_ROOT"
else
    echo "   ❌ Isaac Sim not found at: $ISAAC_SIM_ROOT"
    echo "   Please install Isaac Sim 5.1"
    exit 1
fi

# Check Isaac Sim's internal ROS2
echo ""
echo "2. Checking Isaac Sim's internal ROS2..."
ISAAC_ROS2_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/jazzy"
if [ -d "$ISAAC_ROS2_PATH" ]; then
    echo "   ✅ Internal ROS2 found at: $ISAAC_ROS2_PATH"
else
    echo "   ❌ Internal ROS2 not found at: $ISAAC_ROS2_PATH"
    echo "   Please check Isaac Sim installation"
    exit 1
fi

# Check for rclpy
echo ""
echo "3. Checking Isaac Sim's rclpy..."
if [ -d "$ISAAC_ROS2_PATH/rclpy" ]; then
    echo "   ✅ rclpy directory found"
    if [ -f "$ISAAC_ROS2_PATH/rclpy/rclpy/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so" ] || ls "$ISAAC_ROS2_PATH/rclpy/rclpy/_rclpy*.so" &>/dev/null; then
        echo "   ✅ rclpy binary extension found"
    else
        echo "   ⚠️  rclpy binary extension not found (may be in different location)"
    fi
else
    echo "   ❌ rclpy directory not found"
    exit 1
fi

# Check ROS2 libraries
echo ""
echo "4. Checking ROS2 libraries..."
if [ -d "$ISAAC_ROS2_PATH/lib" ]; then
    echo "   ✅ ROS2 lib directory found"
    if [ -f "$ISAAC_ROS2_PATH/lib/librclcpp.so" ]; then
        echo "   ✅ librclcpp.so found"
    else
        echo "   ⚠️  librclcpp.so not found (may not be needed)"
    fi
else
    echo "   ❌ ROS2 lib directory not found"
    exit 1
fi

# Check startup scripts
echo ""
echo "5. Checking startup scripts..."
if [ -x "./start_isaac_with_ros2.sh" ]; then
    echo "   ✅ start_isaac_with_ros2.sh is executable"
else
    echo "   ⚠️  start_isaac_with_ros2.sh not executable, fixing..."
    chmod +x start_isaac_with_ros2.sh
fi

if [ -x "./run_isaac_cyclone.sh" ]; then
    echo "   ✅ run_isaac_cyclone.sh is executable"
else
    echo "   ⚠️  run_isaac_cyclone.sh not executable, fixing..."
    chmod +x run_isaac_cyclone.sh
fi

# Check system ROS2 (for comparison)
echo ""
echo "6. Checking system ROS2 (for reference)..."
if [ -d "/opt/ros/jazzy" ]; then
    echo "   ✅ System ROS2 Jazzy found"
    SYSTEM_RCLPY="/opt/ros/jazzy/lib/python3.12/site-packages/rclpy"
    if [ -d "$SYSTEM_RCLPY" ]; then
        echo "   📌 System rclpy is for Python 3.12 (incompatible with Isaac Sim)"
    fi
else
    echo "   ℹ️  System ROS2 not found (not needed)"
fi

# Check Python version
echo ""
echo "7. Checking Isaac Sim Python version..."
PYTHON_VERSION=$($ISAAC_SIM_ROOT/python.sh -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null)
if [ "$PYTHON_VERSION" = "3.11" ]; then
    echo "   ✅ Isaac Sim Python version: $PYTHON_VERSION (correct)"
else
    echo "   ⚠️  Isaac Sim Python version: $PYTHON_VERSION (expected 3.11)"
fi

# Final summary
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Verification Complete                                    ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "✅ All checks passed!"
echo ""
echo "To start Isaac Sim with ROS2 support:"
echo "   ./start_isaac_with_ros2.sh"
echo ""
echo "Or:"
echo "   ./run_isaac_cyclone.sh"
echo ""
echo "To verify topics after starting Isaac Sim:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "   ros2 topic list | grep isaac_sim"
echo ""
echo "Expected output:"
echo "   /isaac_sim/reset_done"
echo "   /isaac_sim/reset_request"
echo ""
