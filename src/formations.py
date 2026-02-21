"""
formations.py — Formation Blueprints & Slot Calculator
=======================================================
Defines all formation patterns and provides GPS slot calculation
for any number of drones.
"""

import math

EARTH_R = 6378137.0

# Formation blueprints: list of (forward_offset, right_offset) relative to leader
# Positive forward = ahead of leader, positive right = right of leader
FORMATIONS = {
    'V': {
        'name': 'V-Formation',
        'icon': '◁',
        'description': 'Classic chevron — optimal for aerodynamic efficiency',
        'slots': [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2), (-3,-3), (-3,3)],
    },
    'ARROW': {
        'name': 'Arrow',
        'icon': '⬆',
        'description': 'Spearhead penetration formation',
        'slots': [(0,0), (-1,-1), (-1,1), (-2,0), (-3,-1), (-3,1), (-4,0)],
    },
    'CIRCLE': {
        'name': 'Circle',
        'icon': '◯',
        'description': '360° coverage — ideal for area surveillance',
        'slots': [(0,0)] + [
            (math.cos(i * math.pi / 3), math.sin(i * math.pi / 3))
            for i in range(6)
        ],
    },
    'WALL': {
        'name': 'Wall',
        'icon': '▬',
        'description': 'Lateral barrier — maximum cross-section coverage',
        'slots': [(0,-3), (0,-2), (0,-1), (0,0), (0,1), (0,2), (0,3)],
    },
    'LINE': {
        'name': 'Line',
        'icon': '│',
        'description': 'Trail formation — single-file column',
        'slots': [(0,0), (-1,0), (-2,0), (-3,0), (-4,0), (-5,0), (-6,0)],
    },
}

FORMATION_KEYS = list(FORMATIONS.keys())


def get_slot_gps(base_lat, base_lon, heading_rad, slot_fwd, slot_right, spacing_m):
    """
    Calculate GPS position for a formation slot.
    
    Args:
        base_lat, base_lon: Formation center (leader target) GPS
        heading_rad: Swarm heading in radians (0=North, π/2=East)
        slot_fwd, slot_right: Slot offset in formation units
        spacing_m: Distance in meters between formation units
    
    Returns:
        (latitude, longitude) tuple
    """
    # Rotate slot by heading
    d_north = (slot_fwd * math.cos(heading_rad) - slot_right * math.sin(heading_rad)) * spacing_m
    d_east = (slot_fwd * math.sin(heading_rad) + slot_right * math.cos(heading_rad)) * spacing_m
    
    # Convert meters to GPS offset
    new_lat = base_lat + (d_north / EARTH_R) * (180.0 / math.pi)
    new_lon = base_lon + (d_east / (EARTH_R * math.cos(math.pi * base_lat / 180.0))) * (180.0 / math.pi)
    
    return new_lat, new_lon


def get_formation_slots(formation_name, num_drones):
    """Get slot positions for a given formation, truncated to num_drones."""
    if formation_name not in FORMATIONS:
        formation_name = 'V'
    slots = FORMATIONS[formation_name]['slots']
    return slots[:num_drones]
