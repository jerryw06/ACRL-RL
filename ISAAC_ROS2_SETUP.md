# Isaac Sim ROS2 Setup - SOLVED ✅

## The Problem (Now Fixed)

Isaac Sim couldn't load ROS2, giving this error:
```
[IsaacSim] ROS 2 (rclpy) not available in this Python environment. Teleport service disabled.
```

**Root Cause**: Python version mismatch!
- Isaac Sim uses **Python 3.11**
- System ROS2 Jazzy is compiled for **Python 3.12**
- When environment variables leaked from `.bashrc`, Isaac Sim tried to load Python 3.12's rclpy → failed

## The Solution (Now Implemented)

Isaac Sim **already has** its own internal ROS2 Jazzy at:
```
~/isaacsim_5_1_pegasus/exts/isaacsim.ros2.bridge/jazzy/
```

The fix: Use `env -i` to start with a **clean environment** and only add what's needed, preventing system ROS2 (Python 3.12) from contaminating Isaac Sim's Python 3.11.

## ✅ How to Use (Simple)

The wrapper scripts now handle everything automatically:

**Option 1: Standard CycloneDDS (Recommended)**
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_with_ros2.sh
```

**Option 2: Explicit CycloneDDS script**
```bash
./run_isaac_cyclone.sh
```

## Easy Way: Use the Wrapper Script

I created `start_isaac_with_ros2.sh` that does this for you:

```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_with_ros2.sh
```

This script:
1. Sets the required ROS2 environment variables
2. Sources `/opt/ros/jazzy/setup.bash`
3. Starts Isaac Sim with proper ROS2 access

## How to Run Training (Correct Way)

**Terminal 1 - Isaac Sim with ROS2:**
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_with_ros2.sh
```

Wait until you see:
```
[IsaacSim][ROS2] Topics ready: /isaac_sim/reset_request (sub), /isaac_sim/reset_done (pub)
```

**Terminal 2 - Training:**
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training.sh --isaac-reset
```

## Verify ROS2 Topics

After starting Isaac Sim with the wrapper, check that topics are created:

```bash
ros2 topic list | grep isaac_sim
```

You should see:
```
/isaac_sim/reset_done
/isaac_sim/reset_request
```

## What Happens When It Works

When ROS2 is properly configured, you'll see:
```
[IsaacSim][ROS2] Services ready: /isaac_sim/reset_drone, /isaac_sim/reset_drone_precise, /isaac_sim/restart_px4
[IsaacSim][ROS2] Topics ready: /isaac_sim/reset_request (sub), /isaac_sim/reset_done (pub)
```

Then during training, resets will work:
```
[RESET] Publishing reset request to /isaac_sim/reset_request...
[RESET] Waiting for reset confirmation on /isaac_sim/reset_done...
[IsaacSim][ROS2] *** TOPIC RECEIVED: /isaac_sim/reset_request ***
[IsaacSim][ROS2] *** PUBLISHED: /isaac_sim/reset_done ***
[RESET-SUCCESS] Isaac Sim environment reset confirmed!
```

## Common Mistakes

❌ **DON'T DO THIS**:
```bash
python3 1_px4_single_vehicle_copy.py  # Won't have ROS2!
```

✅ **DO THIS INSTEAD**:
```bash
./start_isaac_with_ros2.sh  # Has ROS2!
```

## Testing the Reset Manually

Once Isaac Sim is running with ROS2, you can test the reset from terminal:

```bash
# Send a reset request
ros2 topic pub --once /isaac_sim/reset_request std_msgs/msg/Bool "{data: true}"

# Watch for confirmation (in another terminal)
ros2 topic echo /isaac_sim/reset_done
```

You should see the drone reset in Isaac Sim and get a confirmation message!

## Summary

The key requirement: **Isaac Sim needs ROS2 environment variables set before it starts.**

Use `start_isaac_with_ros2.sh` and everything will work!
