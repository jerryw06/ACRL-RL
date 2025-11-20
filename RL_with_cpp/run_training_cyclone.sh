#!/bin/bash
# C++ training script configured for CycloneDDS (to match Isaac Sim)

cd ~/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp

# Match Isaac Sim's configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source ~/PegasusSimulator_51/examples/RL_test/ACRL-RL/ros2_local_ws/install/setup.bash

# Source LibTorch environment
source setup_env.sh

echo "=== C++ Trainer Configuration ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

# Run training
ros2 run rl_with_cpp train_rl
