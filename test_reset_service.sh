#!/bin/bash
# Quick test to verify Isaac Sim reset service is reachable

echo "=== Testing Isaac Sim Reset Service ==="
echo ""

export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash

echo "1. Checking if service exists..."
ros2 service list | grep isaac_sim

echo ""
echo "2. Calling /isaac_sim/reset_and_restart_px4..."
ros2 service call /isaac_sim/reset_and_restart_px4 std_srvs/srv/Empty

echo ""
echo "Done! Check Isaac Sim terminal for '[IsaacSim][ROS2] *** SERVICE CALLED:' message"
