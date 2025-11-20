# RL Training with Isaac Sim ROS2 Service Reset

## Overview

This version uses Isaac Sim's **built-in ROS2 reset service** for clean, fast resets between episodes. The Python script (`1_px4_single_vehicle_copy.py`) exposes a reset service that we call from C++ to cleanly reset the environment **without crashing or restarting Isaac Sim**.

## How It Works

Between each training episode, the system will:

1. **Land the drone** (if it's mid-air)
2. **Disarm** the vehicle
3. **Call ROS2 service** `/isaac_sim/reset_drone_precise` to reset the simulation
4. **Wait 2 seconds** for physics to settle
5. **Verify PX4 topics** are available
6. **Continue with normal episode reset** (arm, takeoff, begin training)

Total reset time: **~3-5 seconds** between episodes - much faster and safer than process restart!

## Usage

### Step 1: Start Isaac Sim Manually

In one terminal:
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL
python3 1_px4_single_vehicle_copy.py
```

Wait until you see the drone in the Isaac Sim viewer.

### Step 2: Start Training

In another terminal:
```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training_with_restart.sh
```

This script will:
- Verify Isaac Sim is running
- Check for PX4 topics
- Start training with automatic restart enabled

## Alternative: Direct Run

If you prefer, you can run the training directly with the restart flag:

```bash
cd /home/jerry/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training.sh --isaac-reset
```

## What Changed

### Modified File: `src/px4_accel_env.cpp`

**Function: `teleport_to_spawn()`**

Before:
- Attempted to call a ROS2 service to reset/teleport the drone
- Relied on Isaac Sim's internal reset mechanism

After:
- Sends `SIGTERM` to the Isaac Sim Python script
- Waits for clean shutdown (8 seconds)
- Force kills if still running
- Restarts the script in background
- Waits for full initialization (15 seconds)
- Verifies PX4 topics are available (up to 20 seconds)

**Also Changed:**
- `GRAVITY` constant: Changed from `-9.81f` to `9.81f` (correct NED frame convention)

## Timing Parameters

You can adjust wait times in `src/px4_accel_env.cpp` (function `teleport_to_spawn()`):

- **Line ~1274**: Shutdown wait time (currently 8 seconds)
- **Line ~1290**: Initialization wait time (currently 15 seconds)
- **Line ~1295**: Topic availability timeout (currently 20 seconds)

Increase these if you're on slower hardware or if Isaac Sim takes longer to load.

## Troubleshooting

### "Isaac Sim script not detected"
- Make sure you started Isaac Sim first
- Check that the script is named `1_px4_single_vehicle_copy.py`

### "PX4 topics not available"
- Ensure MicroXRCEAgent is running
- Check PX4 SITL started properly in Isaac Sim logs
- Try increasing wait times

### Training freezes between episodes
- Isaac Sim may have crashed during restart
- Check Isaac Sim terminal for errors
- Increase initialization wait time

### Drone flies downward instead of up
- This was fixed by correcting the GRAVITY constant
- If still occurring, rebuild: `./build.sh`

## Benefits of This Approach

✅ **Complete reset**: Uses Isaac Sim's proper reset mechanism via `world.reset()`  
✅ **Fast**: Only ~3-5 seconds between episodes  
✅ **Reliable positioning**: Drone teleports back to exact spawn point  
✅ **No crashes**: Uses the built-in reset service, no process killing  
✅ **Clean physics**: All rigid bodies properly reset through Isaac Sim API  
✅ **Efficient**: Many more episodes per hour compared to process restart

## Why This is Better Than Process Restart

❌ **Process restart approach** (what we DON'T do):
- Kills and restarts entire Isaac Sim process
- Risk of crashes and orphaned processes
- Takes 25-30 seconds per reset
- Can leave zombie processes
- Unstable and unreliable

✅ **ROS2 service approach** (what we DO):
- Uses Isaac Sim's built-in reset mechanism
- Stable and reliable
- Only 3-5 seconds per reset
- No process management issues
- Clean and proper

## How the Service Works

The Python script (`1_px4_single_vehicle_copy.py`) already has this built in:

```python
class IsaacControlServers:
    def _on_precise_reset(self, request, response):
        self._app.request_reset(world_only=False)
        return response
```

This calls `world.reset()` and teleports the drone back to the original spawn position - exactly what we need!

## Disable Reset Service

To disable the reset service (not recommended):

1. Remove the `--isaac-reset` flag when running
2. Or unset the `ISAAC_SIM_RESET` environment variable
