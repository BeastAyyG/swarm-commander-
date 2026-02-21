#!/bin/bash

# ==============================================================================
# UNIFIED SWARM LAUNCHER
# Executes the entire SITL + Swarm Agent + Commander stack in one command.
# ==============================================================================

NUM_DRONES=3
BASE_DIR="/home/q/swarm compute /sitl_integration"

echo "================================================="
echo "   🚀 INITIALIZING FULL SWARM STACK 🚀          "
echo "================================================="

# 1. Clean up stale processes
echo "[1/4] Cleaning process space..."
pkill -9 -f arducopter
pkill -9 -f drone_agent.py
pkill -9 -f swarm_commander.py
sleep 2

# 2. Launch SITL
echo "[2/4] Spawning $NUM_DRONES ArduPilot SITL instances..."
cd "$BASE_DIR" || exit
./launch_sitl_swarm.sh $NUM_DRONES

# 3. Launch Drone Agents in background
echo "[3/4] Connecting Edge AI Agents to SITL instances..."
for ((i=1; i<=$NUM_DRONES; i++)); do
    PORT=$((5760 + (i-1)*10))
    echo "      -> Starting Agent $i on port $PORT"
    python3 drone_agent.py --id $i --connect "tcp:127.0.0.1:$PORT" > "agent_$i.log" 2>&1 &
done

# 4. Launch Swarm Commander
echo "[4/4] Starting Swarm Commander Uplink..."
echo "      (Waiting for EKF/GPS stability...)"
sleep 10
python3 swarm_commander.py --drones $NUM_DRONES
