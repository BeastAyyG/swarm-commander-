#!/usr/bin/env python3
"""
Swarm Commander — Core Package

This package provides the core functionality for drone swarm coordination:
- formations: Formation blueprints and GPS slot calculations
- avoidance: Artificial Potential Field collision avoidance
- inspection: Multi-drone structural inspection planning
"""

from .formations import (
    FORMATIONS,
    FORMATION_KEYS,
    get_formation_slots,
    get_slot_gps,
)
from .avoidance import APFEngine
from .inspection import (
    Structure,
    InspectionConfig,
    StructuralInspectionPlanner,
    InspectionPhase,
    ScanPoint,
    DroneAssignment,
)

__all__ = [
    # Formations
    "FORMATIONS",
    "FORMATION_KEYS",
    "get_formation_slots",
    "get_slot_gps",
    # Avoidance
    "APFEngine",
    # Inspection
    "Structure",
    "InspectionConfig",
    "StructuralInspectionPlanner",
    "InspectionPhase",
    "ScanPoint",
    "DroneAssignment",
]
