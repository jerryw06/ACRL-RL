#!/bin/bash
# All-in-one launcher: Start Isaac Sim and Training together
# This script manages both processes for you

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Cleanup function
cleanup() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║  Shutting down...                                        ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    
    # Kill training if running
    if [ ! -z "$TRAINING_PID" ]; then
        echo "Stopping training process..."
        kill $TRAINING_PID 2>/dev/null
    fi
    
    # Kill Isaac Sim if we started it
    if [ ! -z "$ISAAC_PID" ]; then
        echo "Stopping Isaac Sim..."
        kill $ISAAC_PID 2>/dev/null
    fi
    
    # Cleanup any remaining processes
    pkill -f "1_px4_single_vehicle_copy.py" 2>/dev/null
    
    echo "Cleanup complete."
    exit 0
}

trap cleanup EXIT INT TERM

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Complete RL Training Launcher                           ║"
echo "║  (Isaac Sim + Training with Restart Mode)                ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Step 1: Start Isaac Sim
echo "[1/4] Starting Isaac Sim..."
cd "$REPO_ROOT"
python3 1_px4_single_vehicle_copy.py > /tmp/isaac_sim_output.log 2>&1 &
ISAAC_PID=$!

echo "      Isaac Sim PID: $ISAAC_PID"
echo "      Logs: /tmp/isaac_sim_output.log"
echo ""

# Step 2: Wait for Isaac Sim and PX4 to be ready
echo "[2/4] Waiting for Isaac Sim and PX4 to initialize..."
echo "      This usually takes 20-30 seconds..."

TIMEOUT=60
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    # Check if Isaac Sim process is still alive
    if ! kill -0 $ISAAC_PID 2>/dev/null; then
        echo "[ERROR] Isaac Sim process died! Check logs:"
        echo "        tail /tmp/isaac_sim_output.log"
        exit 1
    fi
    
    # Check for PX4 topics
    if ros2 topic list 2>/dev/null | grep -q "/fmu/vehicle_local_position/out"; then
        echo "[OK] PX4 topics detected!"
        break
    fi
    
    # Show progress
    if [ $((ELAPSED % 5)) -eq 0 ]; then
        echo "      Still waiting... ($ELAPSED/$TIMEOUT seconds)"
    fi
    
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "[ERROR] PX4 topics not available after $TIMEOUT seconds!"
    echo "[ERROR] Check Isaac Sim logs: tail /tmp/isaac_sim_output.log"
    exit 1
fi

echo ""

# Step 3: Additional settling time
echo "[3/4] Letting simulation stabilize for 5 more seconds..."
sleep 5
echo ""

# Step 4: Start training
echo "[4/4] Starting RL training..."
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Training Starting - Press Ctrl+C to stop                ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

cd "$SCRIPT_DIR"
./run_training.sh --isaac-reset "$@"
TRAINING_PID=$!

# Wait for training to complete or be interrupted
wait $TRAINING_PID
