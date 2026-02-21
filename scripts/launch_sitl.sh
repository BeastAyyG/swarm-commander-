#!/bin/bash

# ==============================================================================
# Swarm SITL Orchestrator (Fixed)
# Launches multiple ArduCopter SITL instances for swarm network testing.
# Usage: ./launch_sitl_swarm.sh <number_of_drones>
# ==============================================================================

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <number_of_drones>"
    echo "Example: $0 3"
    exit 1
fi

NUM_DRONES=$1
BASE_SYSID=1

# Clean up any existing arducopter SITL processes
echo "Killing any existing SITL instances..."
pkill -9 -f arducopter 2>/dev/null
sleep 2

echo "Launching $NUM_DRONES SITL instances..."

for ((i=1; i<=$NUM_DRONES; i++)); do
    SYSID=$((BASE_SYSID + i - 1))
    INSTANCE_ID=$((i - 1))
    
    # Create isolated directory for EEPROM and logs
    DIR="sitl_swarm_instance_$i"
    rm -rf "$DIR"
    mkdir -p "$DIR"
    cd "$DIR" || exit
    
    # Default parameters file - bypass ALL calibration checks
    cat <<EOF > default.parm
SYSID_THISMAV $SYSID
FRAME_CLASS 1
FRAME_TYPE 1
ARMING_CHECK 0
SIM_SPEEDUP 1
BRD_SAFETYENABLE 0
BATT_MONITOR 0
DISARM_DELAY 0
EOF

    echo "=========================================="
    echo "Spawning Drone ID: $i | SYSID: $SYSID | Instance: $INSTANCE_ID"
    echo "DroneKit TCP Port: tcp:127.0.0.1:$((5760 + INSTANCE_ID * 10))"
    echo "=========================================="

    # Launch WITHOUT -w flag (we already wipe by deleting the directory)
    # The --defaults flag loads our params on first boot without the internal restart
    /home/q/ardupilot/build/sitl/bin/arducopter \
        -I$INSTANCE_ID \
        --model quad \
        --home -35.363261,149.165230,584,353 \
        --defaults default.parm \
        > sitl.log 2>&1 &
    
    cd ..
done

echo ""
echo "All $NUM_DRONES instances launched!"
echo "Ports:"
for ((i=1; i<=$NUM_DRONES; i++)); do
    echo "  Drone $i: tcp:127.0.0.1:$((5760 + (i-1) * 10))"
done
