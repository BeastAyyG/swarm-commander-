#!/usr/bin/env python3
"""
swarm_controller.py — High-Level Swarm Controller

Provides a unified interface for controlling drone swarms, whether
simulated or connected to real SITL/hardware.
"""

import math
from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field

from .core.formations import FORMATIONS, get_formation_slots, get_slot_gps
from .core.avoidance import APFEngine
from .agents.simulated_drone import SimulatedDrone
from .utils.config import get_config, SwarmConfig


@dataclass
class SwarmState:
    """Current state of the swarm."""
    num_drones: int = 0
    formation: str = "V"
    spacing: float = 15.0
    waypoint_x: float = 0.0
    waypoint_y: float = 0.0
    cruise_alt: float = 20.0
    heading: float = 0.0  # radians
    active: bool = False
    phase: str = "IDLE"  # IDLE, TAKEOFF, MISSION, RTL, LAND
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "num_drones": self.num_drones,
            "formation": self.formation,
            "spacing": self.spacing,
            "waypoint_x": self.waypoint_x,
            "waypoint_y": self.waypoint_y,
            "cruise_alt": self.cruise_alt,
            "heading": self.heading,
            "active": self.active,
            "phase": self.phase,
        }


class SwarmController:
    """
    High-level controller for drone swarms.
    
    This class provides a unified interface for controlling multiple drones
    in formation, supporting both simulated and real (SITL/hardware) drones.
    
    Features:
        - Formation flight control (V, Arrow, Circle, Wall, Line, Grid)
        - Waypoint navigation
        - Collision avoidance via Artificial Potential Fields
        - Takeoff and landing coordination
        - Real-time state monitoring
    
    Usage:
        >>> controller = SwarmController(num_drones=4, formation="V")
        >>> controller.takeoff(20.0)
        >>> controller.goto(100.0, 50.0)
        >>> controller.set_formation("CIRCLE")
        >>> controller.step()  # Update all drones
    """
    
    def __init__(
        self,
        num_drones: int = 3,
        formation: str = "V",
        spacing: float = 15.0,
        use_simulated: bool = True,
    ):
        """
        Initialize the swarm controller.
        
        Args:
            num_drones: Number of drones in the swarm
            formation: Initial formation name
            spacing: Distance between drones in meters
            use_simulated: If True, use simulated drones; otherwise expect real connections
        """
        self.cfg = get_config()
        self.state = SwarmState(
            num_drones=num_drones,
            formation=formation if formation in FORMATIONS else "V",
            spacing=spacing,
        )
        
        self.drones: List[SimulatedDrone] = []
        self.apf_engine = APFEngine()
        
        if use_simulated:
            self._create_simulated_drones(num_drones)
    
    def _create_simulated_drones(self, n: int) -> None:
        """Create n simulated drones in formation positions."""
        slots = get_formation_slots(self.state.formation, n)
        
        for i in range(n):
            fwd, right = slots[i] if i < len(slots) else (-i, 0)
            
            # Calculate initial position based on formation slot
            d_north = fwd * self.state.spacing
            d_east = right * self.state.spacing
            
            drone = SimulatedDrone(
                drone_id=i + 1,
                initial_x=d_east,
                initial_y=d_north,
            )
            self.drones.append(drone)
    
    def takeoff(self, altitude: float = 20.0) -> List[str]:
        """
        Command all drones to take off.
        
        Args:
            altitude: Target altitude AGL in meters
            
        Returns:
            List of status messages
        """
        messages = []
        self.state.cruise_alt = altitude
        self.state.phase = "TAKEOFF"
        
        for drone in self.drones:
            if drone.arm_and_takeoff(altitude):
                messages.append(f"Drone {drone.drone_id}: Taking off to {altitude}m")
            else:
                messages.append(f"Drone {drone.drone_id}: Already airborne")
        
        self.state.active = True
        self._assign_formation_targets()
        self.state.phase = "MISSION"
        
        return messages
    
    def land(self) -> List[str]:
        """
        Command all drones to land.
        
        Returns:
            List of status messages
        """
        messages = []
        self.state.phase = "LAND"
        
        for drone in self.drones:
            if drone.land():
                messages.append(f"Drone {drone.drone_id}: Landing")
            else:
                messages.append(f"Drone {drone.drone_id}: Already grounded")
        
        return messages
    
    def goto(
        self,
        x: float,
        y: float,
        alt: Optional[float] = None,
    ) -> List[str]:
        """
        Command the swarm to navigate to a waypoint.
        
        Args:
            x: Target east offset from home (meters)
            y: Target north offset from home (meters)
            alt: Optional target altitude AGL (meters)
            
        Returns:
            List of status messages
        """
        # Update heading based on direction of travel
        dx = x - self.state.waypoint_x
        dy = y - self.state.waypoint_y
        if math.hypot(dx, dy) > 0.1:
            self.state.heading = math.atan2(dx, dy)
        
        self.state.waypoint_x = x
        self.state.waypoint_y = y
        
        if alt is not None:
            self.state.cruise_alt = alt
        
        self._assign_formation_targets()
        
        airborne_count = sum(1 for d in self.drones if d.state.value == "AIRBORNE")
        return [f"Navigating {airborne_count} drones to ({x:.1f}, {y:.1f})"]
    
    def set_formation(self, name: str) -> str:
        """
        Change the swarm formation.
        
        Args:
            name: Formation name (V, ARROW, CIRCLE, WALL, LINE, GRID)
            
        Returns:
            Status message
        """
        name = name.upper()
        if name not in FORMATIONS:
            return f"Unknown formation '{name}'. Available: {list(FORMATIONS.keys())}"
        
        old = self.state.formation
        self.state.formation = name
        self._assign_formation_targets()
        
        return f"Formation changed: {old} → {name}"
    
    def add_drones(self, n: int = 1) -> List[str]:
        """
        Add drones to the swarm.
        
        Args:
            n: Number of drones to add
            
        Returns:
            List of status messages
        """
        messages = []
        slots = get_formation_slots(self.state.formation, self.state.num_drones + n)
        
        for i in range(n):
            new_id = len(self.drones) + 1
            slot_idx = new_id - 1
            fwd, right = slots[slot_idx] if slot_idx < len(slots) else (-slot_idx, 0)
            
            d_north = fwd * self.state.spacing
            d_east = right * self.state.spacing
            
            drone = SimulatedDrone(
                drone_id=new_id,
                initial_x=d_east,
                initial_y=d_north,
            )
            self.drones.append(drone)
            messages.append(f"Added Drone {new_id}")
        
        self.state.num_drones += len(self.drones)
        self._assign_formation_targets()
        
        return messages
    
    def remove_drone(self, drone_id: int) -> str:
        """
        Remove a drone from the swarm.
        
        Args:
            drone_id: ID of the drone to remove
            
        Returns:
            Status message
        """
        for i, drone in enumerate(self.drones):
            if drone.drone_id == drone_id:
                self.drones.pop(i)
                self.state.num_drones -= 1
                return f"Removed Drone {drone_id}"
        
        return f"Drone {drone_id} not found"
    
    def step(self, dt: float = 0.1) -> None:
        """
        Advance the simulation by one time step.
        
        Args:
            dt: Time step in seconds
        """
        for drone in self.drones:
            drone.step(dt)
    
    def _assign_formation_targets(self) -> None:
        """Assign formation slot targets to all drones."""
        n = len(self.drones)
        slots = get_formation_slots(self.state.formation, n)
        
        for i, drone in enumerate(self.drones):
            fwd, right = slots[i] if i < len(slots) else (-i, 0)
            
            # Rotate slot by swarm heading
            d_north = (fwd * math.cos(self.state.heading) - 
                      right * math.sin(self.state.heading)) * self.state.spacing
            d_east = (fwd * math.sin(self.state.heading) + 
                     right * math.cos(self.state.heading)) * self.state.spacing
            
            target_x = self.state.waypoint_x + d_east
            target_y = self.state.waypoint_y + d_north
            
            drone.goto(target_x, target_y, self.state.cruise_alt)
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get comprehensive swarm status.
        
        Returns:
            Dictionary containing swarm state and individual drone statuses
        """
        return {
            "swarm_state": self.state.to_dict(),
            "drones": [
                {
                    "id": d.drone_id,
                    "state": d.state.value,
                    "position": d.get_position(),
                    "target": d.get_target(),
                    "status": d.get_status().__dict__,
                }
                for d in self.drones
            ],
        }
    
    def __repr__(self) -> str:
        return (
            f"SwarmController(drones={len(self.drones)}, "
            f"formation={self.state.formation}, "
            f"phase={self.state.phase})"
        )
