#!/bin/bash
# ⚠️  THIS SCRIPT IS NO LONGER NEEDED ⚠️
#
# Isaac Sim already has ROS2 Jazzy built-in at:
#   ~/isaacsim_5_1_pegasus/exts/isaacsim.ros2.bridge/jazzy/
#
# The startup scripts (start_isaac_with_ros2.sh and run_isaac_cyclone.sh)
# now use Isaac Sim's internal ROS2 to avoid Python version conflicts.

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  ⚠️  This script is deprecated                            ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Isaac Sim ALREADY has ROS2 Jazzy built-in!"
echo ""
echo "Location: ~/isaacsim_5_1_pegasus/exts/isaacsim.ros2.bridge/jazzy/"
echo ""
echo "The startup scripts now use Isaac Sim's internal ROS2 automatically."
echo "No additional installation needed."
echo ""
echo "To run Isaac Sim with ROS2:"
echo "  ./start_isaac_with_ros2.sh"
echo ""
echo "Or:"
echo "  ./run_isaac_cyclone.sh"
echo ""

exit 0
