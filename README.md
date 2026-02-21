<p align="center">
  <h1 align="center">🚁 Swarm Commander</h1>
  <p align="center">
    <strong>Autonomous Multi-Drone Formation Control with Real-Time Obstacle Avoidance</strong>
  </p>
  <p align="center">
    <a href="#features"><img src="https://img.shields.io/badge/Drones-5+-blue?style=for-the-badge" /></a>
    <a href="#features"><img src="https://img.shields.io/badge/ArduPilot-SITL-orange?style=for-the-badge" /></a>
    <a href="#features"><img src="https://img.shields.io/badge/Python-3.10+-green?style=for-the-badge" /></a>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge" /></a>
  </p>
</p>

---

## 📋 Overview

**Swarm Commander** is a full-stack drone swarm simulation and control system that integrates with [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) for realistic flight dynamics. It features **DJI-style touchscreen waypoint control**, **Artificial Potential Field (APF) collision avoidance**, **5 dynamic formation types**, and a **real-time 2D visualization** built with Pygame.

This project demonstrates production-grade swarm coordination concepts applicable to search-and-rescue, agricultural surveying, and defense scenarios.

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🎮 **Interactive Waypoint Control** | Click-to-fly DJI-style interface with waypoint queuing and auto-heading |
| 🛡️ **APF Collision Avoidance** | Real-time Artificial Potential Fields prevent inter-drone and obstacle collisions |
| 📐 **5 Formation Types** | V, Arrow, Circle, Wall, Line — switch instantly with hotkeys |
| 🚁 **ArduPilot SITL Integration** | Real ArduCopter firmware with MAVLink communication via DroneKit |
| 📡 **Live Telemetry HUD** | Real-time altitude, speed, GPS position, mode, and fleet status |
| 🗺️ **Radar Minimap** | Overview of entire operational area with drone and obstacle positions |
| 🎯 **Multi-Waypoint Mission** | Queue multiple waypoints for autonomous mission execution |
| 🔴 **Dynamic Obstacles** | Right-click to place obstacles; drones organically route around them |
| ⚡ **Random Spawn & Gather** | Drones start at random positions and autonomously gather into formation |
| 🎨 **Pure Pygame Simulation** | Standalone simulations that work without ArduPilot for rapid prototyping |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────┐
│                  SWARM COMMANDER                     │
│  ┌───────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ Waypoint  │  │Formation │  │    APF Engine    │  │
│  │  Queue    │──│ Planner  │──│ (Avoid+Attract)  │  │
│  └───────────┘  └──────────┘  └──────────────────┘  │
│         │              │               │             │
│         ▼              ▼               ▼             │
│  ┌─────────────────────────────────────────────┐     │
│  │          DroneKit / MAVLink Bridge           │     │
│  └─────────────────────────────────────────────┘     │
│         │         │         │         │         │    │
│         ▼         ▼         ▼         ▼         ▼    │
│     ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐│
│     │SITL 1│  │SITL 2│  │SITL 3│  │SITL 4│  │SITL 5││
│     │:5760 │  │:5770 │  │:5780 │  │:5790 │  │:5800 ││
│     └──────┘  └──────┘  └──────┘  └──────┘  └──────┘│
└─────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
swarm-commander/
├── README.md
├── LICENSE
├── requirements.txt
├── setup.sh                     # One-click environment setup
├── src/
│   ├── interactive_commander.py # Full SITL + Pygame interactive demo
│   ├── unified_swarm.py         # Headless SITL mission runner
│   ├── avoidance.py             # APF collision avoidance engine
│   ├── formations.py            # Formation blueprints & slot calculator
│   └── sim/
│       ├── basic_swarm.py       # Minimal swarm logic demo
│       ├── visual_swarm.py      # Pygame-only swarm visualization
│       ├── advanced_swarm.py    # Self-healing + boids + bidding
│       └── missile_evasion.py   # Missile threat evasion simulation
├── scripts/
│   └── launch_sitl.sh           # Launch multiple SITL instances
├── docs/
│   ├── SETUP.md                 # Detailed setup guide
│   ├── ARCHITECTURE.md          # Technical deep-dive
│   └── CONTROLS.md              # User controls reference
├── tests/
│   └── test_sitl_connection.py  # SITL connectivity test
└── assets/
    └── demo.gif                 # Demo recording (placeholder)
```

---

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- [ArduPilot SITL](https://ardupilot.org/dev/docs/building-setup-linux.html) built for `arducopter`
- Pygame, DroneKit, pymavlink

### 1. Clone & Setup

```bash
git clone https://github.com/YOUR_USERNAME/swarm-commander.git
cd swarm-commander
chmod +x setup.sh && ./setup.sh
```

### 2. Run Interactive Commander (Full Demo)

```bash
python3 -u src/interactive_commander.py
```

This launches 5 SITL instances, connects DroneKit agents, calibrates & arms all drones, then opens the interactive Pygame window.

### 3. Run Pure Simulation (No ArduPilot Required)

```bash
python3 src/sim/visual_swarm.py
```

---

## 🎮 Controls

| Input | Action |
|-------|--------|
| **Left-Click** | Set waypoint — swarm flies there in formation |
| **Right-Click** | Place obstacle — drones route around it |
| **1 – 5** | Switch formation (V / Arrow / Circle / Wall / Line) |
| **Scroll** | Zoom in/out |
| **C** | Clear waypoint queue |
| **X** | Clear all obstacles |
| **F** | Toggle camera auto-follow |
| **ESC** | Quit & cleanup |

---

## 🧠 Technical Details

### Artificial Potential Fields (APF)

The avoidance system uses a real-time APF algorithm:

- **Attractive Force**: Pulls each drone toward its formation slot
- **Repulsive Force (Drones)**: Pushes drones apart when within `AVOID_RADIUS` (14m)
- **Repulsive Force (Obstacles)**: Pushes drones away from obstacles within `OBSTACLE_REPULSE` (30m)

```python
F_total = F_attractive + Σ F_repulsive_drones + Σ F_repulsive_obstacles
```

### Formation Slot Assignment

Each formation is defined as a list of `(forward, right)` offset slots relative to the leader:

```python
FORMATIONS = {
    'V':     [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2)],
    'ARROW': [(0,0), (-1,-1), (-1,1), (-2,0), (-3,-1), (-3,1)],
    ...
}
```

Slots are rotated by the swarm heading and scaled by `SPACING` (18m) to produce GPS waypoints.

### SITL Integration

- **Binary**: Raw `arducopter` binary execution (no `sim_vehicle.py` dependency)
- **Calibration**: MAVLink `MAV_CMD_PREFLIGHT_CALIBRATION` sent programmatically
- **Communication**: DroneKit TCP connections with retry logic
- **Parameters**: Minimal params that bypass all safety checks for simulation

---

## 🎬 Demos

### Interactive Commander
> 5 drones spawn at random positions, gather into V-formation, then follow your click waypoints while avoiding obstacles.

### Missile Evasion Simulation
> Swarm detects incoming threats and uses APF to organically scatter and regroup.

### Self-Healing Formation
> Drones with low battery return to charge; the swarm redistributes slots using an auction algorithm.

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Max Drones Tested | 7 |
| Avoidance Update Rate | 2 Hz |
| Visualization FPS | 30 |
| Waypoint Reach Threshold | 8m |
| Min Inter-Drone Distance | 14m |
| SITL Init Time | ~90s (5 drones) |

---

## 🗺️ Roadmap

- [ ] 3D visualization with OpenGL/Cesium
- [ ] Real hardware deployment (Pixhawk)
- [ ] ROS2 integration
- [ ] Multi-agent reinforcement learning for path planning
- [ ] Web-based ground control station
- [ ] Swarm-to-swarm adversarial scenarios

---

## 🤝 Contributing

Contributions are welcome! Please open an issue or PR.

---

## 📄 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

<p align="center">
  Built with ❤️ for autonomous systems research
</p>
