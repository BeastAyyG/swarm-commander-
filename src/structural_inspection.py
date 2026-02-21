"""
structural_inspection.py — Autonomous Multi-Drone Structural Inspection
=========================================================================
Coordinates a swarm to inspect a structure (building, bridge, tower, etc.)
by dividing it into sectors and altitude bands. Each drone autonomously
orbits its assigned sector from top to bottom, capturing full coverage.

Key Features:
  - Automatic sector division (N drones → N sectors of 360°/N each)
  - Altitude banding (top-to-bottom sweep for full façade coverage)
  - Coordinated simultaneous scanning (all drones move in parallel)
  - Orbit path generation at configurable radius
  - Overlap management for image stitching
  - Real-time progress tracking per drone

Use Cases:
  - Building façade inspection
  - Bridge/dam structural monitoring
  - Cell tower / wind turbine scanning
  - Post-disaster damage assessment
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum


class InspectionPhase(Enum):
    IDLE = "IDLE"
    POSITIONING = "POSITIONING"       # Moving to start position
    SCANNING = "SCANNING"             # Active inspection orbit
    DESCENDING = "DESCENDING"         # Moving to next altitude band
    RETURNING = "RETURNING"           # Flying back to home
    COMPLETE = "COMPLETE"


@dataclass
class Structure:
    """Defines the structure to be inspected."""
    center_x: float = 0.0            # meters (local frame)
    center_y: float = 0.0
    radius: float = 20.0             # radius of the structure (meters)
    height: float = 50.0             # total height (meters)
    name: str = "Structure"
    shape: str = "cylinder"           # cylinder, rectangle
    width: float = 0.0               # for rectangular structures
    depth: float = 0.0


@dataclass
class InspectionConfig:
    """Configuration for the inspection mission."""
    orbit_radius: float = 35.0       # Distance from structure center
    alt_top: float = 50.0            # Starting altitude (top)
    alt_bottom: float = 10.0         # Ending altitude (bottom)
    alt_bands: int = 5               # Number of altitude layers
    orbit_points_per_sector: int = 8  # Waypoints per sector arc
    scan_speed: float = 3.0          # m/s during inspection
    camera_overlap_pct: float = 20.0  # % overlap between sectors
    dwell_time_s: float = 2.0        # Time to hover at each scan point


@dataclass
class ScanPoint:
    """A single scan waypoint."""
    x: float                         # meters
    y: float
    alt: float                       # meters AGL
    heading: float                   # radians, pointing toward structure
    sector: int
    band: int                        # altitude band index (0=top)
    point_idx: int                   # index within the sector


@dataclass
class DroneAssignment:
    """Inspection assignment for a single drone."""
    drone_id: int
    sector_id: int
    sector_start_angle: float        # radians
    sector_end_angle: float
    scan_points: List[ScanPoint] = field(default_factory=list)
    current_point_idx: int = 0
    current_band: int = 0
    phase: InspectionPhase = InspectionPhase.IDLE
    progress_pct: float = 0.0
    photos_taken: int = 0
    total_points: int = 0

    @property
    def current_target(self) -> Optional[ScanPoint]:
        if self.current_point_idx < len(self.scan_points):
            return self.scan_points[self.current_point_idx]
        return None

    def advance(self):
        """Move to the next scan point."""
        self.current_point_idx += 1
        self.photos_taken += 1
        if self.total_points > 0:
            self.progress_pct = (self.current_point_idx / self.total_points) * 100.0
        if self.current_point_idx >= len(self.scan_points):
            self.phase = InspectionPhase.COMPLETE


class StructuralInspectionPlanner:
    """
    Plans and coordinates multi-drone structural inspection.
    
    Divides the structure into N equal sectors (one per drone),
    generates orbit scan points at multiple altitude bands,
    and provides real-time coordination.
    """

    def __init__(self, structure: Structure, config: InspectionConfig, num_drones: int):
        self.structure = structure
        self.config = config
        self.num_drones = num_drones
        self.assignments: List[DroneAssignment] = []
        self.phase = InspectionPhase.IDLE
        self.start_time: Optional[float] = None
        self.overall_progress: float = 0.0

        self._plan_mission()

    def _plan_mission(self):
        """Generate scan points for all drones across all altitude bands."""
        sector_angle = (2 * math.pi) / self.num_drones
        overlap_angle = math.radians(self.config.camera_overlap_pct / 100.0 * 
                                      math.degrees(sector_angle))

        # Altitude bands (top to bottom)
        alt_step = (self.config.alt_top - self.config.alt_bottom) / max(1, self.config.alt_bands - 1)
        altitudes = [self.config.alt_top - i * alt_step for i in range(self.config.alt_bands)]

        for drone_idx in range(self.num_drones):
            # Sector boundaries
            start_angle = drone_idx * sector_angle - overlap_angle / 2
            end_angle = (drone_idx + 1) * sector_angle + overlap_angle / 2

            scan_points = []
            for band_idx, alt in enumerate(altitudes):
                # Generate orbit points within this sector at this altitude
                # Alternate direction for even/odd bands (serpentine pattern)
                angles = self._sector_angles(start_angle, end_angle, 
                                              self.config.orbit_points_per_sector)
                if band_idx % 2 == 1:
                    angles = list(reversed(angles))

                for pt_idx, angle in enumerate(angles):
                    px = self.structure.center_x + self.config.orbit_radius * math.cos(angle)
                    py = self.structure.center_y + self.config.orbit_radius * math.sin(angle)
                    # Heading: point toward structure center
                    heading = math.atan2(self.structure.center_y - py,
                                         self.structure.center_x - px)
                    
                    scan_points.append(ScanPoint(
                        x=px, y=py, alt=alt,
                        heading=heading,
                        sector=drone_idx,
                        band=band_idx,
                        point_idx=pt_idx
                    ))

            assignment = DroneAssignment(
                drone_id=drone_idx + 1,
                sector_id=drone_idx,
                sector_start_angle=start_angle,
                sector_end_angle=end_angle,
                scan_points=scan_points,
                total_points=len(scan_points),
            )
            self.assignments.append(assignment)

    def _sector_angles(self, start, end, num_points) -> List[float]:
        """Generate evenly spaced angles within a sector."""
        if num_points <= 1:
            return [(start + end) / 2]
        step = (end - start) / (num_points - 1)
        return [start + i * step for i in range(num_points)]

    def start_inspection(self):
        """Begin the inspection mission."""
        self.phase = InspectionPhase.SCANNING
        self.start_time = time.time()
        for a in self.assignments:
            a.phase = InspectionPhase.POSITIONING
            a.current_point_idx = 0

    def get_drone_target(self, drone_id: int) -> Optional[Tuple[float, float, float, float]]:
        """
        Get the current target for a drone.
        Returns (x, y, alt, heading) or None if inspection is complete.
        """
        for a in self.assignments:
            if a.drone_id == drone_id:
                target = a.current_target
                if target:
                    return (target.x, target.y, target.alt, target.heading)
                return None
        return None

    def report_arrived(self, drone_id: int, current_x: float, current_y: float, threshold: float = 3.0):
        """
        Called when a drone reports its position. 
        If close enough to target, advances to next scan point.
        Returns True if the drone should take a photo.
        """
        for a in self.assignments:
            if a.drone_id == drone_id:
                target = a.current_target
                if target is None:
                    return False
                
                dist = math.sqrt((current_x - target.x)**2 + (current_y - target.y)**2)
                if dist < threshold:
                    a.phase = InspectionPhase.SCANNING
                    a.advance()
                    self._update_progress()
                    return True  # Take photo
        return False

    def _update_progress(self):
        """Recalculate overall inspection progress."""
        total = sum(a.total_points for a in self.assignments)
        done = sum(a.photos_taken for a in self.assignments)
        self.overall_progress = (done / total * 100) if total > 0 else 0

        if all(a.phase == InspectionPhase.COMPLETE for a in self.assignments):
            self.phase = InspectionPhase.COMPLETE

    def get_status_summary(self) -> dict:
        """Get a comprehensive status report."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        return {
            'phase': self.phase.value,
            'progress_pct': self.overall_progress,
            'elapsed_s': elapsed,
            'total_photos': sum(a.photos_taken for a in self.assignments),
            'total_points': sum(a.total_points for a in self.assignments),
            'drones': [{
                'id': a.drone_id,
                'sector': a.sector_id,
                'phase': a.phase.value,
                'progress': a.progress_pct,
                'photos': a.photos_taken,
                'band': a.current_band if a.current_target else -1,
                'remaining': a.total_points - a.current_point_idx,
            } for a in self.assignments],
        }

    def get_all_scan_points(self) -> List[ScanPoint]:
        """Get all scan points across all assignments for visualization."""
        all_pts = []
        for a in self.assignments:
            all_pts.extend(a.scan_points)
        return all_pts

    def get_sector_boundaries(self) -> List[Tuple[float, float, float, float]]:
        """Get sector boundary lines for visualization. Returns (x1,y1,x2,y2) pairs."""
        lines = []
        for a in self.assignments:
            # Line from structure center to orbit radius at sector start
            x1 = self.structure.center_x
            y1 = self.structure.center_y
            x2 = x1 + self.config.orbit_radius * 1.2 * math.cos(a.sector_start_angle)
            y2 = y1 + self.config.orbit_radius * 1.2 * math.sin(a.sector_start_angle)
            lines.append((x1, y1, x2, y2))
        return lines
