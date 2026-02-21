# 🎮 Controls Reference

## Mouse Controls

| Input | Action |
|-------|--------|
| **Left-Click** | Set waypoint — swarm flies to clicked position in current formation |
| **Right-Click** | Place obstacle (6m radius) — drones will avoid it with APF |
| **Scroll Up** | Zoom in |
| **Scroll Down** | Zoom out |
| **Hover** | Shows distance from leader to cursor position |

## Keyboard Controls

| Key | Action |
|-----|--------|
| **1** | V-Formation (chevron) |
| **2** | Arrow formation (spearhead) |
| **3** | Circle formation (360° coverage) |
| **4** | Wall formation (lateral barrier) |
| **5** | Line formation (single-file column) |
| **C** | Clear all queued waypoints |
| **X** | Clear all obstacles |
| **F** | Toggle camera auto-follow mode |
| **ESC** | Quit and cleanup all SITL processes |
| **+/-** | Alternative zoom controls |

## Waypoint System

- Click multiple locations to **queue** waypoints
- The swarm advances to the next waypoint when the leader reaches the current one (within 8m)
- Heading is automatically computed from the direction of travel
- Dashed lines show the planned route through all queued waypoints
- The current target shows a pulsing red crosshair

## HUD Elements

- **Top Bar**: Mission phase, formation selector, elapsed time
- **Right Panel**: Per-drone telemetry (mode, altitude, position, speed, status)
- **Bottom Bar**: Controls reference, obstacle count
- **Radar (Bottom-Left)**: Minimap showing all drones and obstacles
- **Formation Lines**: Green = safe, Yellow = warning, Red = too close
