#!/bin/bash
# Script to run RL training with Isaac Sim script restart between episodes
# This ensures a clean reset by stopping and restarting the Isaac Sim Python script

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  RL Training with Isaac Sim Script Restart               ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "This training mode will:"
echo "  1. Start with Isaac Sim already running"
echo "  2. Between episodes, STOP and RESTART Isaac Sim completely"
echo "  3. Wait for initialization before continuing"
echo ""
echo "IMPORTANT: Start Isaac Sim manually BEFORE running this script:"
echo "  cd $REPO_ROOT"
echo "  python3 1_px4_single_vehicle_copy.py"
echo ""
read -p "Press Enter when Isaac Sim is running and drone is visible..."

# Check if Isaac Sim is actually running
if ! pgrep -f "1_px4_single_vehicle_copy.py" > /dev/null; then
    echo "[ERROR] Isaac Sim script not detected!"
    echo "[ERROR] Please start it first:"
    echo "  cd $REPO_ROOT"
    echo "  python3 1_px4_single_vehicle_copy.py"
    exit 1
fi

echo "[OK] Isaac Sim detected as running."
echo ""

# Wait for PX4 topics to be available
echo "Checking for PX4 topics..."
TIMEOUT=30
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/vehicle_local_position/out"; then
        echo "[OK] PX4 topics detected!"
        break
    fi
    echo "  Waiting for PX4 topics... ($ELAPSED/$TIMEOUT seconds)"
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "[ERROR] PX4 topics not available after $TIMEOUT seconds!"
    echo "[ERROR] Make sure PX4 SITL and MicroXRCEAgent are running."
    exit 1
fi

echo ""
echo "Starting RL training with Isaac Sim restart enabled..."
echo "The script will automatically stop and restart Isaac Sim between episodes."
echo ""

# Run training with Isaac Sim reset flag enabled
cd "$SCRIPT_DIR"
./run_training.sh --isaac-reset "$@"
