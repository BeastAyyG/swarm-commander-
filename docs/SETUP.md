# 🛠️ Setup Guide

## Prerequisites

### 1. Python 3.10+
```bash
python3 --version  # Must be 3.10 or higher
```

### 2. ArduPilot SITL (for full integration)

```bash
# Clone ArduPilot
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot

# Install dependencies
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduCopter SITL
./waf configure --board sitl
./waf copter

# Verify
~/ardupilot/build/sitl/bin/arducopter --help
```

### 3. Python Dependencies

```bash
pip3 install -r requirements.txt
```

> **Note:** DroneKit has a compatibility issue with Python 3.10+. The `interactive_commander.py` 
> script automatically patches this with a `collections.abc` monkey-patch.

## Running

### Full SITL Demo (Recommended)
```bash
python3 -u src/interactive_commander.py
```

This will:
1. Launch 5 ArduCopter SITL instances
2. Wait 15 seconds for EKF initialization
3. Connect DroneKit agents to each instance
4. Calibrate accelerometers via MAVLink
5. Arm and takeoff all drones to 20m
6. Open the interactive Pygame window

### Pure Simulation (No ArduPilot)
```bash
python3 src/sim/visual_swarm.py
python3 src/sim/advanced_swarm.py
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Connection refused` | SITL needs more time. Wait 15-20 seconds after launch |
| `3D Accel calibration needed` | The script sends MAVLink calibration automatically. If it persists, increase the post-calibration wait |
| `collections.MutableMapping` | Already patched in the scripts. If you see this in other tools, add the monkey-patch at the top of your script |
| `Port already in use` | Run `pkill -9 -f arducopter` to kill stale instances |
