# 🏗️ System Architecture

## Overview

Swarm Commander uses a **centralized planning, decentralized execution** architecture. 
A single Commander process calculates formation waypoints, while each drone independently 
applies Artificial Potential Field (APF) avoidance to navigate safely.

## Components

### 1. Swarm Commander (Planning Layer)
- Manages the waypoint queue
- Calculates formation slot GPS positions
- Auto-computes swarm heading from movement direction
- Detects waypoint arrival and advances the queue

### 2. APF Avoidance Engine (Execution Layer)
- Each drone computes its own repulsive forces
- Forces combine attractive (toward slot) and repulsive (away from neighbors/obstacles)
- Runs at 2Hz update rate
- Produces modified GPS targets sent to ArduPilot

### 3. SITL Bridge (Communication Layer)
- TCP connections to ArduCopter SITL via DroneKit
- MAVLink heartbeats, telemetry parsing, and command encoding
- Automatic accelerometer calibration (MAV_CMD_PREFLIGHT_CALIBRATION)
- Retry logic for robust connection handling

### 4. Visualization (Presentation Layer)
- Pygame 2D top-down renderer at 30 FPS
- Camera system with smooth lerp tracking
- Trail rendering with time-based fade
- Spinning prop animation
- Interactive obstacle and waypoint placement

## Data Flow

```
User Click → Waypoint Queue → Formation Slot Calculator
                                       ↓
                              GPS Target per Drone
                                       ↓
                              APF Force Combination
                                       ↓
                              Modified GPS Target
                                       ↓
                     DroneKit simple_goto() → MAVLink
                                       ↓
                          ArduCopter SITL Firmware
                                       ↓
                        Simulated Physics & Telemetry
                                       ↓
                          DroneKit Telemetry Callback
                                       ↓
                            Pygame Visualization
```

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `SPACING` | 18m | Distance between formation slots |
| `AVOID_RADIUS` | 14m | Minimum inter-drone distance |
| `OBSTACLE_REPULSE` | 30m | Obstacle detection range |
| `CRUISE_ALT` | 20m | Operating altitude |
| `WP_REACH_DIST` | 8m | Distance to consider waypoint reached |

## Threading Model

```
Main Thread (Pygame)
├── Event loop (mouse/keyboard)
├── Camera update
├── Drone state polling
└── Rendering

Init Thread
├── SITL connection
├── Calibration
├── Arm & Takeoff
└── Sets commander.ready = True

Avoidance Thread (2Hz loop)
├── Read commander.current_wp
├── Calculate formation slots
├── Apply APF for each drone
└── Send simple_goto() commands
```
