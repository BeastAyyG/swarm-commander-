"""
formations.py — Formation Blueprints, Slot Calculator & Formation Morphing
===========================================================================
Defines all formation patterns, provides GPS slot calculation for any
number of drones, and implements smooth animated interpolation between
formations (formation morphing).

Formation morphing
------------------
``FormationMorpher`` computes per-drone interpolated slot positions
between a source and target formation using linear (lerp) interpolation
over a configurable number of steps.  Each step is a list of
``(forward_offset, right_offset)`` tuples — one per drone — that can be
sent directly to :func:`get_slot_gps` to move drones smoothly.

Usage::

    morpher = FormationMorpher(num_drones=4)
    frames = morpher.morph("V", "CIRCLE", steps=20)
    for frame in frames:
        for drone_idx, (fwd, right) in enumerate(frame):
            lat, lon = get_slot_gps(base_lat, base_lon, heading, fwd, right, spacing)
            vehicles[drone_idx].simple_goto(LocationGlobalRelative(lat, lon, alt))
        time.sleep(0.1)
"""

import math
from typing import List, Optional, Tuple

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
    'GRID': {
        'name': 'Grid',
        'icon': '⊞',
        'description': 'Rectangular grid — maximum area search coverage',
        'slots': [
            (0, 0), (0, 2), (0, -2),
            (-2, 0), (-2, 2), (-2, -2),
            (-4, 0),
        ],
    },
}

FORMATION_KEYS = list(FORMATIONS.keys())


def get_slot_gps(
    base_lat: float,
    base_lon: float,
    heading_rad: float,
    slot_fwd: float,
    slot_right: float,
    spacing_m: float,
) -> Tuple[float, float]:
    """
    Calculate GPS position for a formation slot.

    Args:
        base_lat, base_lon: Formation center (leader target) GPS coordinates.
        heading_rad: Swarm heading in radians (0 = North, π/2 = East).
        slot_fwd, slot_right: Slot offset in formation units.
        spacing_m: Distance in metres between formation units.

    Returns:
        ``(latitude, longitude)`` tuple.
    """
    # Rotate slot by heading
    d_north = (slot_fwd * math.cos(heading_rad) - slot_right * math.sin(heading_rad)) * spacing_m
    d_east = (slot_fwd * math.sin(heading_rad) + slot_right * math.cos(heading_rad)) * spacing_m

    # Convert metres to GPS offset
    new_lat = base_lat + (d_north / EARTH_R) * (180.0 / math.pi)
    new_lon = base_lon + (d_east / (EARTH_R * math.cos(math.pi * base_lat / 180.0))) * (180.0 / math.pi)

    return new_lat, new_lon


def get_formation_slots(formation_name: str, num_drones: int) -> List[Tuple[float, float]]:
    """
    Get slot positions for a given formation, truncated to *num_drones*.

    Args:
        formation_name: Key in :data:`FORMATIONS` (falls back to ``'V'`` if unknown).
        num_drones:     Number of drones to assign slots.

    Returns:
        List of ``(forward_offset, right_offset)`` tuples, one per drone.
    """
    if formation_name not in FORMATIONS:
        formation_name = 'V'
    slots = FORMATIONS[formation_name]['slots']
    return slots[:num_drones]


class FormationMorpher:
    """
    Smooth animated interpolation between two formation shapes.

    Generates a sequence of per-drone slot positions that linearly
    interpolate between the source and target formation over a given
    number of steps.  The caller drives the animation by iterating
    over the returned frames at the desired frame rate.

    Args:
        num_drones: Number of drones in the swarm.
    """

    def __init__(self, num_drones: int) -> None:
        """
        Initialise the morpher.

        Args:
            num_drones: Number of drones participating in the morph.
        """
        self.num_drones = num_drones
        self._current_formation: Optional[str] = None

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        """
        Linear interpolation between *a* and *b*.

        Args:
            a: Start value.
            b: End value.
            t: Interpolation factor in [0, 1].

        Returns:
            Interpolated float value.
        """
        return a + (b - a) * t

    def _pad_slots(
        self,
        slots: List[Tuple[float, float]],
        n: int,
    ) -> List[Tuple[float, float]]:
        """
        Pad or truncate a slot list to exactly *n* entries.

        Short lists are padded by repeating the last slot.

        Args:
            slots: Raw slot list from :data:`FORMATIONS`.
            n:     Desired length.

        Returns:
            List of exactly *n* ``(fwd, right)`` tuples.
        """
        if len(slots) >= n:
            return list(slots[:n])
        # Pad with last slot repeated
        padded = list(slots)
        while len(padded) < n:
            padded.append(slots[-1])
        return padded

    def morph(
        self,
        from_formation: str,
        to_formation: str,
        steps: int = 20,
    ) -> List[List[Tuple[float, float]]]:
        """
        Generate interpolated frames between two formations.

        Each frame is a list of ``(fwd, right)`` slot offsets, one per
        drone.  Frame 0 is identical to *from_formation*; the final frame
        is identical to *to_formation*.

        Args:
            from_formation: Source formation name (key in :data:`FORMATIONS`).
            to_formation:   Target formation name.
            steps:          Total number of interpolation frames (≥ 2).

        Returns:
            List of *steps* frames.  Each frame is a list of
            ``(fwd_offset, right_offset)`` tuples.

        Raises:
            ValueError: If *steps* < 2.
        """
        if steps < 2:
            raise ValueError("steps must be >= 2")

        src_slots = self._pad_slots(
            get_formation_slots(from_formation, self.num_drones),
            self.num_drones,
        )
        dst_slots = self._pad_slots(
            get_formation_slots(to_formation, self.num_drones),
            self.num_drones,
        )

        frames: List[List[Tuple[float, float]]] = []
        for step in range(steps):
            t = step / (steps - 1)
            frame: List[Tuple[float, float]] = []
            for (sf, sr), (df, dr) in zip(src_slots, dst_slots):
                frame.append((
                    self._lerp(sf, df, t),
                    self._lerp(sr, dr, t),
                ))
            frames.append(frame)

        self._current_formation = to_formation
        return frames

    @property
    def current_formation(self) -> Optional[str]:
        """Name of the most recently targeted formation, or None."""
        return self._current_formation

