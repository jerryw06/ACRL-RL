# Multi-Drone Parallel RL Training Guide

## Current Status (November 20, 2025)

### ✅ What's Working
- **Parallel multi-drone training** - 5 drones training simultaneously
- **PX4 SITL integration** - Each drone has dedicated MicroXRCEAgent
- **Isaac Sim integration** - Physics simulation and environment
- **Improved takeoff sequence** - Stable 5-second climb with verification
- **Stricter reset validation** - Position and velocity checks before RL control

### ⚠️ Known Issues
- **Reset mechanism not 100% reliable** - Isaac Sim topic-based reset occasionally times out
- **Position drift after reset** - Some episodes start with small offset from spawn
- **Takeoff stabilization** - Occasionally requires multiple attempts to reach stable hover

## How to Start Training (PROPER SEQUENCE)

### Prerequisites
```bash
# 1. ROS2 Jazzy installed
# 2. PX4-Autopilot SITL
# 3. Isaac Sim 4.0+ with Pegasus Simulator
# 4. LibTorch 2.1.0
# 5. CycloneDDS (preferred) or FastDDS
```

### Step 1: Start Isaac Sim with Multi-Drone Scene
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_isaac_multi_drone.sh
```

**Wait for Isaac Sim to fully load** - You should see:
- 5 drones spawned in the scene
- Physics simulation running
- ROS2 bridge initialized

### Step 2: Start MicroXRCEAgent Bridges (5 instances)
```bash
# In a new terminal
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./start_micro_ros_agents.sh
```

This starts 5 MicroXRCEAgent instances on ports 8888-8892.

### Step 3: Start Multi-Drone Training
```bash
# In a new terminal
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
./run_multi_drone_training_only.sh
```

**OR** if Isaac Sim + MicroXRCEAgent are already running:
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training.sh
```

## Reset Mechanism Details

### How It Should Work
1. **Disarm** - Drone disarms and lands if mid-air
2. **Teleport** - Isaac Sim teleports drone back to spawn (via ROS2 topic)
3. **Physics Reset** - Velocities zeroed out
4. **Position Settle** - Wait for stable position (< 2cm drift, < 0.05 m/s velocity)
5. **Re-arm & Takeoff** - Smooth 5-second climb to 2.5m altitude
6. **Hover Stabilization** - Wait for stable hover (30cm tolerance, 0.2 m/s velocity)
7. **RL Verification** - Final check before handing control to RL agent

### Current Reset Issues

**Problem:** Isaac Sim reset confirmation timeout
```
[RESET-POS] ❌ TIMEOUT - No confirmation after 5s
```

**Possible Causes:**
1. Isaac Sim not running or crashed
2. ROS2 bridge not initialized in Isaac Sim
3. Reset callback not subscribed to topic
4. DDS/RMW mismatch between C++ and Python

**Debugging Commands:**
```bash
# Check if reset topics exist
ros2 topic list | grep reset

# Monitor reset requests (in another terminal while training)
ros2 topic echo /isaac_sim/reset_request_0

# Monitor reset confirmations
ros2 topic echo /isaac_sim/reset_done_0

# Check if Python script is subscribed
ros2 topic info /isaac_sim/reset_request_0
```

### Reset Validation Criteria

**Position Settling (3s timeout):**
- Position drift: < 2cm per check
- Velocity magnitude: < 0.05 m/s
- Must be stable for 3 consecutive checks (300ms)

**Velocity Zeroing (5s timeout):**
- Velocity magnitude: < 0.05 m/s
- Must be stable for 5 consecutive checks (250ms)

**Hover Stabilization (20s timeout):**
- Altitude error: < 30cm
- Vertical velocity: < 0.2 m/s
- Must be stable for 1.5 seconds

**RL-Ready Verification (10s timeout):**
- Altitude error: < 30cm
- All velocities: < 0.2 m/s
- Must be stable for 0.5 seconds

## Improved Takeoff Sequence

The takeoff has been significantly improved for stability:

1. **Longer Climb Duration** - 5 seconds (was 3s) for smooth ascent
2. **Progress Logging** - Shows climb progress at 20%, 40%, 60%, 80%, 100%
3. **Relaxed Tolerances** - More realistic thresholds for hover stability
4. **Better Error Messages** - Clear indication of what's failing

**Expected Takeoff Output:**
```
[TAKEOFF] Progress: 20% - current z=-0.5 m, target z=-0.5 m, vz=-0.3 m/s
[TAKEOFF] Progress: 40% - current z=-1.0 m, target z=-1.0 m, vz=-0.4 m/s
[TAKEOFF] Progress: 60% - current z=-1.5 m, target z=-1.5 m, vz=-0.3 m/s
[TAKEOFF] Progress: 80% - current z=-2.0 m, target z=-2.0 m, vz=-0.2 m/s
[TAKEOFF] Progress: 100% - current z=-2.5 m, target z=-2.5 m, vz=-0.1 m/s
[HOVER] ✓ STABLE for 1.5s!
[RL-READY-CHECK] Starting position VERIFIED - RL takeover in 1 second...
```

## Training Parameters

**Environment:**
- Episode length: 6 seconds
- Control frequency: 200 Hz
- Max acceleration: 3.0 m/s²
- Target altitude: 2.0 m
- Target lateral distance: 2.0 m

**PPO Hyperparameters:**
- Learning rate: 3e-4
- Discount factor (γ): 0.99
- GAE lambda: 0.95
- Clip epsilon: 0.2
- Mini-batch size: 64
- Update epochs: 10

## Troubleshooting

### Training Won't Start
```bash
# Check PX4 topics
ros2 topic list | grep fmu

# Check if MicroXRCEAgent is running
pgrep -f MicroXRCEAgent

# Check if Isaac Sim is running
pgrep -f 2_px4_multi_vehicle.py
```

### Reset Fails
```bash
# Restart Isaac Sim
./start_isaac_multi_drone.sh

# Wait 30 seconds for full initialization

# Restart training
./run_multi_drone_training_only.sh
```

### Drones Not Arming
- Check PX4 parameters in `px4_offboard_params.txt`
- Verify MicroXRCEAgent ports (8888-8892)
- Check RMW_IMPLEMENTATION (should be CycloneDDS)

### Build Errors
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./build.sh
```

## Architecture

### Multi-Drone Setup
- 5 PX4AccelEnv instances (one per drone)
- Each with dedicated vehicle_id (1-5)
- Separate ROS2 namespaces: `/px4_0/fmu/*`, `/px4_1/fmu/*`, etc.
- Separate reset topics: `/isaac_sim/reset_request_0`, `/isaac_sim/reset_done_0`, etc.

### Reset Communication
```
C++ Trainer                Isaac Sim (Python)
    |                             |
    |--[reset_request_X]-------->|
    |                          [world.reset()]
    |                          [teleport drone]
    |<--[reset_done_X]-----------|
    |                             |
    [wait for settle]             |
    [verify position]             |
    [re-arm & takeoff]            |
```

## Next Steps / TODO

1. **Fix Isaac Sim reset reliability** - Investigate why confirmation messages are lost
2. **Reduce reset validation timeouts** - Make episode resets faster
3. **Add reset fallback mechanism** - Manual position reset if teleport fails
4. **Improve hover PID tuning** - Reduce oscillations during stabilization
5. **Add episode metrics logging** - Track reset success rate, takeoff time, etc.

## Files Modified (This Commit)

- `RL_with_cpp/src/px4_accel_env.cpp` - Improved reset validation and takeoff sequence
- `RL_with_cpp/src/px4_node.cpp` - Vehicle-specific topic namespaces
- `RL_with_cpp/include/px4_accel_env.hpp` - Reset function declarations
- `RL_with_cpp/include/px4_node.hpp` - Multi-drone support

## Performance Notes

**Training Speed:**
- ~200 Hz control loop (5ms per step)
- ~6 second episodes
- ~1200 steps per episode
- Reset overhead: 15-25 seconds (when working properly)

**Hardware Requirements:**
- CPU: 8+ cores recommended
- GPU: RTX 3060+ for Isaac Sim
- RAM: 32GB+ recommended
- Storage: SSD for fast episode logging
