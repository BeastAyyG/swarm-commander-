<p align="center">
  <h1 align="center">🚁 Swarm Commander</h1>
  <p align="center">
    <strong>Autonomous Multi-Drone Structural Inspection & Formation Control</strong>
  </p>
  <p align="center">
    <a href="#structural-inspection"><img src="https://img.shields.io/badge/🏗️_Structural-Inspection-blue?style=for-the-badge" /></a>
    <a href="#features"><img src="https://img.shields.io/badge/Drones-Up_to_7-green?style=for-the-badge" /></a>
    <a href="#features"><img src="https://img.shields.io/badge/ArduPilot-SITL-orange?style=for-the-badge" /></a>
    <a href="#features"><img src="https://img.shields.io/badge/Python-3.10+-yellow?style=for-the-badge" /></a>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-lightgrey?style=for-the-badge" /></a>
  </p>
</p>

---

## 📋 Overview

**Swarm Commander** is a full-stack drone swarm platform for **autonomous structural inspection** and **formation flight control**, integrated with [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).

The headline feature is **Multi-Drone Structural Inspection** — a coordinated system where drones divide a structure into sectors and simultaneously scan each sector from top to bottom, dramatically reducing inspection time. The captured data can be stitched into full 3D façade maps.

---

## 🏗️ Structural Inspection (Key Feature)

### The Problem
Inspecting large structures (buildings, bridges, towers, wind turbines) manually is **slow, expensive, and dangerous**. A single drone requires a skilled operator and covers only one side at a time.

### Our Solution
**Swarm Commander's Structural Inspection** module divides any structure into **N equal sectors** (one per drone) and executes a coordinated top-to-bottom scan in parallel:

```
                    ╭──── Drone 1 (Sector 1: 0°-90°)
                    │
   Structure ───────┼──── Drone 2 (Sector 2: 90°-180°)
   (Tower/          │
    Building)  ─────┼──── Drone 3 (Sector 3: 180°-270°)
                    │
                    ╰──── Drone 4 (Sector 4: 270°-360°)
                    
   Each drone orbits its sector at 5 altitude bands (50m → 10m)
   with 20% overlap for image stitching
```

### How It Works

1. **Sector Division**: 360° / N drones = each drone covers its arc
2. **Altitude Banding**: Structure height divided into layers (top → bottom)
3. **Serpentine Sweep**: Alternating orbit direction per band for efficiency
4. **Camera Overlap**: 20% overlap between adjacent sectors for seamless stitching
5. **Progress Tracking**: Real-time per-drone progress, photo count, and ETA
6. **Parallel Execution**: All drones scan simultaneously → N× faster than single drone

### Run the Demo

```bash
python3 src/inspection_demo.py
```

| Control | Action |
|---------|--------|
| **Left-Click** | Place structure to inspect |
| **SPACE** | Start inspection mission |
| **1-7** | Change number of drones |
| **+/-** | Adjust orbit radius |
| **R** | Reset mission |

### Real-World Applications

| Application | Description |
|-------------|-------------|
| 🏢 **Building Façade** | Inspect exterior walls for cracks, water damage, insulation gaps |
| 🌉 **Bridge Monitoring** | Scan piers, cables, deck underside for structural fatigue |
| 📡 **Cell Tower** | Inspect antenna arrays, cabling, and structural bolts |
| ⚡ **Wind Turbine** | Blade surface scanning for erosion, lightning damage |
| 🏗️ **Construction** | Progress monitoring with time-lapse 3D reconstruction |
| 🆘 **Disaster Response** | Rapid building damage assessment after earthquakes/storms |

---

## ✨ All Features

| Feature | Description |
|---------|-------------|
| 🏗️ **Structural Inspection** | Multi-drone coordinated scanning with sector division and altitude bands |
| 🖥️ **CLI Commander** | Fully interactive terminal CLI — takeoff, land, goto, formation change (no SITL needed) |
| 🎮 **Interactive Waypoint Control** | DJI-style click-to-fly with waypoint queuing and auto-heading |
| 🛡️ **APF Collision Avoidance** | Real-time Artificial Potential Fields prevent inter-drone collisions |
| 📐 **6 Formation Types** | V, Arrow, Circle, Wall, Line, Grid — switch instantly with hotkeys or CLI |
| 🚁 **ArduPilot SITL** | Real ArduCopter firmware with MAVLink + DroneKit |
| 📡 **Live Telemetry** | Real-time altitude, speed, GPS, mode, health status |
| 🗺️ **Radar Minimap** | Fleet overview with obstacle positions |
| 🎯 **Multi-Waypoint Mission** | Queue waypoints for autonomous path following |
| 🔴 **Dynamic Obstacles** | Right-click to place obstacles; drones route around them |
| 🩺 **Health Monitoring** | Motor, battery, IMU, GPS, structural, and comms diagnostics |
| 🧪 **Unit Tests** | 53 tests for formations, APF engine, drone model, and fleet logic |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      SWARM COMMANDER                         │
│                                                              │
│  ┌──────────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │   Structural      │  │  Formation   │  │  APF Avoidance │  │
│  │   Inspection      │  │  Planner     │  │  Engine        │  │
│  │   Planner         │  │              │  │                │  │
│  │  ┌────────────┐   │  │  V / Arrow / │  │  Attractive +  │  │
│  │  │Sector Div. │   │  │  Circle /    │  │  Repulsive     │  │
│  │  │Alt. Bands  │   │  │  Wall / Line │  │  Forces        │  │
│  │  │Serpentine  │   │  │              │  │                │  │
│  │  │Scan Paths  │   │  │              │  │                │  │
│  │  └────────────┘   │  └──────────────┘  └───────────────┘  │
│  └──────────────────┘                                        │
│              │                    │                │          │
│              ▼                    ▼                ▼          │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │            DroneKit / MAVLink Interface                  │ │
│  └─────────────────────────────────────────────────────────┘ │
│         │         │         │         │         │            │
│     ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐       │
│     │SITL 1│  │SITL 2│  │SITL 3│  │SITL 4│  │SITL 5│       │
│     └──────┘  └──────┘  └──────┘  └──────┘  └──────┘       │
└─────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
swarm-commander/
├── README.md
├── LICENSE
├── requirements.txt
├── setup.sh                         # One-click setup
├── src/
│   ├── cli_commander.py             # ⭐ Text CLI — takeoff/land/goto/formation (no SITL needed)
│   ├── interactive_commander.py     # DJI-style SITL + Pygame demo
│   ├── inspection_demo.py           # Structural inspection visual demo
│   ├── structural_inspection.py     # Inspection planner (sectors, bands, paths)
│   ├── unified_swarm.py             # Headless SITL mission runner
│   ├── avoidance.py                 # APF collision avoidance engine
│   ├── formations.py                # Formation blueprints & GPS slots (V/ARROW/CIRCLE/WALL/LINE/GRID)
│   ├── health_monitor.py            # Fleet health monitoring system
│   ├── health_visualizer.py         # Health dashboard Pygame renderer
│   └── sim/
│       ├── basic_swarm.py           # Minimal swarm demo
│       ├── visual_swarm.py          # Pygame swarm visualization
│       ├── advanced_swarm.py        # Self-healing + boids + auction
│       └── missile_evasion.py       # Threat evasion simulation
├── scripts/
│   └── launch_sitl.sh               # Multi-drone SITL launcher
├── docs/
│   ├── SETUP.md
│   ├── ARCHITECTURE.md
│   └── CONTROLS.md
└── tests/
    ├── test_swarm_core.py           # Unit tests (formations, APF, fleet, inspection)
    └── test_sitl_connection.py      # SITL connectivity test
```

---

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- [ArduPilot SITL](https://ardupilot.org/dev/docs/building-setup-linux.html) (for full SITL integration — optional)
- Pygame, DroneKit, pymavlink (optional for visual/SITL modes)

### Setup

```bash
git clone https://github.com/BeastAyyG/swarm-commander-.git
cd swarm-commander-
pip install -r requirements.txt
# or use the one-click script:
chmod +x setup.sh && ./setup.sh
```

### 🖥️ CLI Commander (No ArduPilot, No Display Required)

The **CLI Commander** is the easiest way to get started.  It runs a fully
simulated swarm entirely in the terminal — no ArduPilot, SITL, or GUI needed.

```bash
python3 src/cli_commander.py                          # 3-drone V-formation (default)
python3 src/cli_commander.py --drones 5               # 5 drones
python3 src/cli_commander.py --drones 7 --formation GRID --spacing 20
python3 src/cli_commander.py --batch commands.txt     # batch / scripted mode
```

**Interactive session example:**

```
swarm> status                    # print fleet status table
swarm> takeoff 25                # arm all drones, climb to 25 m
swarm> goto 200 100              # navigate to (200, 100) in local frame
swarm> formation CIRCLE          # switch to circle formation mid-flight
swarm> simulate 30 1             # advance simulation 30 × 1 s steps
swarm> status                    # check new positions
swarm> land                      # land all drones
swarm> exit
```

**All CLI commands:**

| Command | Description |
|---------|-------------|
| `takeoff [alt]` | Arm all drones and climb to altitude (default 20 m) |
| `land` | Land all airborne drones |
| `goto <x> <y> [alt]` | Navigate swarm to local-frame waypoint (metres) |
| `formation <name>` | Change formation: `V`, `ARROW`, `CIRCLE`, `WALL`, `LINE`, `GRID` |
| `formations` | List all formations with descriptions |
| `add [n]` | Add n drones to the fleet (default 1) |
| `remove <id>` | Remove a drone by ID |
| `simulate [steps] [dt]` | Advance physics simulation (default 10 steps × 1 s) |
| `status` | Print fleet status table |
| `reset` | Return all drones to home |
| `help` | Show help for all commands |
| `exit` / `quit` | Quit the CLI |

### 🎮 Visual Simulation Demo (Pygame, No ArduPilot)

```bash
python3 src/sim/visual_swarm.py     # mouse-driven swarm visualization
```

### 🏗️ Structural Inspection Demo (Pygame, No ArduPilot)

```bash
python3 src/inspection_demo.py
```

### Full SITL Interactive Commander (ArduPilot required)

```bash
python3 -u src/interactive_commander.py
```

### 🧪 Run Tests

```bash
pytest tests/test_swarm_core.py -v    # 53 unit tests, no SITL required
```

---

## 🎮 Controls Summary

### Structural Inspection
| Input | Action |
|-------|--------|
| **Left-Click** | Place structure |
| **SPACE** | Start inspection |
| **1-7** | Number of drones |
| **+/-** | Orbit radius |
| **R** | Reset mission |

### Interactive Commander (SITL)
| Input | Action |
|-------|--------|
| **Left-Click** | Set waypoint |
| **Right-Click** | Place obstacle |
| **1-5** | Change formation |
| **C / X** | Clear waypoints / obstacles |
| **F** | Toggle camera follow |

---

## 🧠 Technical Details

### Structural Inspection Algorithm

```python
# For each of N drones:
sector_angle = 360° / N
for altitude in [50m, 40m, 30m, 20m, 10m]:    # Top to bottom
    for point in orbit_arc(sector_start, sector_end):  # Serpentine sweep
        fly_to(point, altitude)
        point_camera_at(structure_center)
        capture_photo()
```

### APF Collision Avoidance

```python
F_total = F_attractive(target) + Σ F_repulsive(drones) + Σ F_repulsive(obstacles)
```

- Drone repulsion: activates within 14m, inverse-distance scaling
- Obstacle repulsion: activates within 30m, higher gain

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Max Drones | 7 |
| Inspection Speed | N× faster (N = drone count) |
| Altitude Bands | 5 (configurable) |
| Sector Overlap | 20% (for stitching) |
| Scan Points/Sector | 10 per band |
| Total Coverage | 250 scan points (5 drones × 5 bands × 10 points) |
| Avoidance Update | 2 Hz |
| Visualization | 60 FPS |

---

## ✅ What's Implemented

- [x] Multi-drone structural inspection
- [x] 6 formation types (V, Arrow, Circle, Wall, Line, **Grid**) with APF avoidance
- [x] ArduPilot SITL integration
- [x] DJI-style interactive waypoint control
- [x] Health monitoring system
- [x] **Text-based CLI commander** (no SITL/display required)
- [x] **Unit test suite** (53 tests — formations, APF, drone model, fleet)

---

## 🔭 Planned / Future Work

The following features are on the roadmap but **not yet implemented**:

- [ ] 3D point cloud generation from scan data
- [ ] Real hardware deployment (Pixhawk + RaspberryPi)
- [ ] ROS2 integration
- [ ] AI-powered defect detection on captured images
- [ ] Web-based ground control station

Contributions and PRs for any of the above are welcome!

---

## 🤝 Contributing

Contributions are welcome! Please open an issue or PR.

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

---

<p align="center">
  Built with ❤️ for autonomous systems research<br/>
  <strong>Structural Inspection • Formation Control • Collision Avoidance</strong>
</p>
