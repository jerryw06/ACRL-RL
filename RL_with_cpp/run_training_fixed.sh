#!/bin/bash
# Fixed training script that matches Isaac Sim's RMW implementation

cd ~/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp

# CRITICAL: Match Isaac Sim's RMW and domain
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/fastdds_profile.xml"

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source ~/PegasusSimulator_51/examples/RL_test/ACRL-RL/ros2_local_ws/install/setup.bash

# Source LibTorch environment
source setup_env.sh

echo "=== Configuration ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "FASTDDS_DEFAULT_PROFILES_FILE: $FASTDDS_DEFAULT_PROFILES_FILE"
echo ""

# Run training
ros2 run rl_with_cpp train_rl
