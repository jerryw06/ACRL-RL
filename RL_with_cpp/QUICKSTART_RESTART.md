# Quick Start Guide - RL Training with Script Restart

## 🚀 Three Ways to Run

### Option 1: Fully Automated (Recommended)
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./start_training_complete.sh
```
This starts everything automatically and handles cleanup.

### Option 2: Manual Control (RECOMMENDED)
**Terminal 1 - Start Isaac Sim with ROS2:**
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_with_ros2.sh
```
⚠️ **IMPORTANT**: Use the wrapper script `start_isaac_with_ros2.sh` which sets up ROS2 environment variables that Isaac Sim needs!

**Terminal 2 (after Isaac Sim loads):**
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training.sh --isaac-reset
```

### Option 3: Direct Run (Expert)
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training.sh --isaac-reset
```

## ⏱️ What to Expect

- **Initial startup**: 20-30 seconds (Isaac Sim + PX4)
- **First episode**: Immediate start after initialization
- **Between episodes**: 3-5 seconds (ROS2 service reset - super fast!)
- **Each episode duration**: ~10 seconds (configurable)

## 📊 Training Progress

You'll see output like:
```
=== Episode 1/200 ===
[RESET] CALLING ISAAC SIM RESET SERVICE FOR CLEAN RESET
[RESET] Waiting for Isaac Sim reset service...
[RESET-SUCCESS] Reset service found!
[RESET] Calling /isaac_sim/reset_drone_precise service...
[RESET-SUCCESS] Isaac Sim environment reset complete!
[RESET] Waiting 2 seconds for physics to settle...
[RESET-SUCCESS] ISAAC SIM RESET COMPLETE
[TAKEOFF-GUARD] RL control begins...
```

## 🎯 Key Features

✅ **Fixed gravity bug** - Drone now flies UP correctly  
✅ **Complete reset** - Fresh simulation each episode  
✅ **Automatic restart** - No manual intervention needed  
✅ **Robust timing** - Enough wait time for initialization  
✅ **Error handling** - Detects failures and reports them  

## 🛠️ Adjusting Wait Times

If you need to adjust timing, edit `RL_with_cpp/src/px4_accel_env.cpp` in function `teleport_to_spawn()`:

- **Service timeout**: `wait_for_service(std::chrono::seconds(5))` - How long to wait for service
- **Reset timeout**: `spin_until_future_complete(..., seconds(10))` - Service call timeout
- **Settle time**: `sleep_for(seconds(2))` - Physics settle time after reset
- **Topic timeout**: `wait_for_local(5.0)` - PX4 topic availability check

Then rebuild: `cd RL_with_cpp && ./build.sh`

## 🐛 Troubleshooting

### ❌ "ROS 2 (rclpy) not available" in Isaac Sim
**Problem**: Isaac Sim can't find ROS2
```
[IsaacSim] ROS 2 (rclpy) not available in this Python environment. Teleport service disabled.
```

**Solution**: Use the wrapper script that sets up ROS2 environment:
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_with_ros2.sh  # ← Use this, NOT python3 1_px4_single...
```

The wrapper sets these critical environment variables:
- `ROS_DISTRO=jazzy`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `LD_LIBRARY_PATH` includes `/opt/ros/jazzy/lib`

### ❌ Reset timeout - no confirmation received
**Problem**: C++ publishes reset request but never receives confirmation
```
[RESET-WARN] Reset confirmation timeout (>10s).
```

**Causes**:
1. Isaac Sim doesn't have ROS2 → Use `start_isaac_with_ros2.sh`
2. Topics not matching → Check with `ros2 topic list | grep isaac_sim`
3. DDS mismatch → Make sure both use same RMW (CycloneDDS recommended)

**Check if topics exist**:
```bash
ros2 topic list | grep isaac_sim
# Should show:
# /isaac_sim/reset_request
# /isaac_sim/reset_done
```

### Isaac Sim won't start
```bash
# Check if it's already running
pgrep -f "1_px4_single_vehicle_copy.py"

# Kill if stuck
pkill -9 -f "1_px4_single_vehicle_copy.py"

# Check Python environment
which python3
```

### PX4 topics not found
```bash
# List all topics
ros2 topic list

# Look for /fmu/* topics
ros2 topic list | grep fmu

# Check if MicroXRCEAgent is running
ps aux | grep MicroXRCEAgent
```

### Training hangs
- Press `Ctrl+C` to stop
- Check `/tmp/isaac_sim_output.log` for errors
- Increase wait times if hardware is slow

### Drone still flies down
- Verify rebuild completed: `cd RL_with_cpp && ./build.sh`
- Check GRAVITY constant is `9.81f` (positive) in `src/px4_accel_env.cpp` line 15

## 📝 Logs and Outputs

- **Isaac Sim**: `/tmp/isaac_sim_output.log` (when using complete launcher)
- **Training**: Directly to terminal
- **Checkpoints**: Saved in `RL_with_cpp/` directory

## 🔄 Stopping Training

Press `Ctrl+C` in the training terminal. The cleanup handler will:
1. Stop the training process
2. Kill Isaac Sim gracefully
3. Clean up any remaining processes

## 📚 More Information

See `RESTART_MODE_README.md` for detailed technical documentation.
