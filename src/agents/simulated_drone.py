#!/usr/bin/env python3
"""
simulated_drone.py — Simulated Drone Agent

A simulated drone model for testing swarm logic without requiring
ArduPilot SITL or hardware. Implements the same interface as real
drone agents for seamless substitution.
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple
from enum import Enum


class DroneState(Enum):
    """Drone flight state."""
    GROUNDED = "GROUNDED"
    AIRBORNE = "AIRBORNE"
    LANDING = "LANDING"
    RTL = "RTL"
    EMERGENCY = "EMERGENCY"


@dataclass
class DroneStatus:
    """Current drone status telemetry."""
    armed: bool = False
    mode: str = "INIT"
    altitude_agl: float = 0.0
    altitude_amsl: float = 584.0
    latitude: float = -35.363261
    longitude: float = 149.165230
    heading: float = 0.0  # degrees
    groundspeed: float = 0.0
    airspeed: float = 0.0
    battery_voltage: float = 22.2
    battery_remaining: float = 100.0
    ekf_ok: bool = True
    health_ok: bool = True


class SimulatedDrone:
    """
    Simulated drone agent for testing and development.
    
    This class implements a physics-based drone simulation that can be
    used in place of real SITL-connected drones for development and testing.
    
    Features:
        - 3D position and velocity simulation
        - State machine (grounded, airborne, landing, etc.)
        - Simple goto navigation
        - Takeoff and landing sequences
        - Telemetry streaming
    
    Usage:
        >>> drone = SimulatedDrone(drone_id=1, home_lat=-35.363261, home_lon=149.165230)
        >>> drone.arm_and_takeoff(20.0)
        >>> drone.goto(local_x=100.0, local_y=50.0, alt=20.0)
        >>> drone.step(dt=0.1)  # Advance simulation by 100ms
        >>> print(drone.get_status())
    """
    
    # Physical parameters
    MAX_HORIZONTAL_SPEED = 15.0  # m/s
    MAX_VERTICAL_SPEED = 5.0     # m/s
    HORIZONTAL_ACCEL = 3.0       # m/s²
    VERTICAL_ACCEL = 2.0         # m/s²
    
    def __init__(
        self,
        drone_id: int,
        home_lat: float = -35.363261,
        home_lon: float = 149.165230,
        home_alt: float = 584.0,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
    ):
        """
        Initialize a simulated drone.
        
        Args:
            drone_id: Unique identifier for this drone
            home_lat: Home latitude (degrees)
            home_lon: Home longitude (degrees)
            home_alt: Home altitude AMSL (meters)
            initial_x: Initial east offset from home (meters)
            initial_y: Initial north offset from home (meters)
        """
        self.drone_id = drone_id
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt
        
        # Position in local frame (meters from home)
        self.x = initial_x
        self.y = initial_y
        self.z = 0.0  # Altitude AGL
        
        # Velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # Target
        self.target_x = initial_x
        self.target_y = initial_y
        self.target_z = 0.0
        
        # State
        self.state = DroneState.GROUNDED
        self.status = DroneStatus()
        self.status.latitude = home_lat
        self.status.longitude = home_lon
        self.status.altitude_amsl = home_alt
        
        # Configuration
        self._armed = False
        self._mode = "INIT"
        self._heading = 0.0  # radians, 0 = North
    
    @property
    def armed(self) -> bool:
        return self._armed
    
    @property
    def mode(self) -> str:
        return self._mode
    
    @property
    def heading(self) -> float:
        return self._heading
    
    def arm_and_takeoff(self, target_alt: float = 20.0) -> bool:
        """
        Arm motors and take off to target altitude.
        
        Args:
            target_alt: Target altitude AGL in meters
            
        Returns:
            True if takeoff initiated successfully
        """
        if self.state != DroneState.GROUNDED:
            return False
        
        self._armed = True
        self._mode = "GUIDED"
        self.state = DroneState.AIRBORNE
        self.target_z = target_alt
        self.status.armed = True
        self.status.mode = "GUIDED"
        
        return True
    
    def land(self) -> bool:
        """
        Initiate landing sequence.
        
        Returns:
            True if landing initiated successfully
        """
        if self.state == DroneState.GROUNDED:
            return False
        
        self.state = DroneState.LANDING
        self.target_z = 0.0
        self._mode = "LAND"
        self.status.mode = "LAND"
        
        return True
    
    def rtl(self) -> bool:
        """
        Return to launch (home position).
        
        Returns:
            True if RTL initiated successfully
        """
        self.state = DroneState.RTL
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 20.0  # Return at 20m first
        self._mode = "RTL"
        self.status.mode = "RTL"
        
        return True
    
    def goto(
        self,
        local_x: Optional[float] = None,
        local_y: Optional[float] = None,
        alt: Optional[float] = None,
    ) -> bool:
        """
        Navigate to a target position.
        
        Args:
            local_x: Target east offset from home (meters)
            local_y: Target north offset from home (meters)
            alt: Target altitude AGL (meters)
            
        Returns:
            True if command accepted
        """
        if not self._armed:
            return False
        
        if local_x is not None:
            self.target_x = local_x
        if local_y is not None:
            self.target_y = local_y
        if alt is not None:
            self.target_z = alt
        
        self._mode = "GUIDED"
        self.status.mode = "GUIDED"
        
        return True
    
    def step(self, dt: float = 0.1) -> None:
        """
        Advance the simulation by one time step.
        
        Args:
            dt: Time step in seconds
        """
        # Horizontal movement
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist_h = math.sqrt(dx * dx + dy * dy)
        
        if dist_h > 0.1:
            # Calculate desired velocity
            desired_vx = (dx / dist_h) * min(dist_h / dt, self.MAX_HORIZONTAL_SPEED)
            desired_vy = (dy / dist_h) * min(dist_h / dt, self.MAX_HORIZONTAL_SPEED)
            
            # Apply acceleration limits
            ax = max(-self.HORIZONTAL_ACCEL, min(desired_vx - self.vx, self.HORIZONTAL_ACCEL))
            ay = max(-self.HORIZONTAL_ACCEL, min(desired_vy - self.vy, self.HORIZONTAL_ACCEL))
            
            self.vx += ax * dt
            self.vy += ay * dt
            
            self.x += self.vx * dt
            self.y += self.vy * dt
            
            # Update heading to face direction of travel
            self._heading = math.atan2(dx, dy)
        else:
            self.x = self.target_x
            self.y = self.target_y
            self.vx = 0.0
            self.vy = 0.0
        
        # Vertical movement
        dz = self.target_z - self.z
        if abs(dz) > 0.1:
            desired_vz = math.copysign(min(abs(dz) / dt, self.MAX_VERTICAL_SPEED), dz)
            az = max(-self.VERTICAL_ACCEL, min(desired_vz - self.vz, self.VERTICAL_ACCEL))
            self.vz += az * dt
            self.z += self.vz * dt
        else:
            self.z = self.target_z
            self.vz = 0.0
        
        # State transitions
        if self.state == DroneState.LANDING and self.z < 0.1:
            self.z = 0.0
            self.state = DroneState.GROUNDED
            self._armed = False
            self._mode = "INIT"
        
        # Update status telemetry
        self._update_status()
    
    def _update_status(self) -> None:
        """Update the status telemetry from current state."""
        # Convert local coords to GPS
        earth_r = 6378137.0
        lat_rad = math.radians(self.home_lat)
        
        dlat = self.y / earth_r * 180.0 / math.pi
        dlon = self.x / (earth_r * math.cos(lat_rad)) * 180.0 / math.pi
        
        self.status.latitude = self.home_lat + dlat
        self.status.longitude = self.home_lon + dlon
        self.status.altitude_agl = self.z
        self.status.altitude_amsl = self.home_alt + self.z
        self.status.heading = math.degrees(self._heading) % 360
        self.status.groundspeed = math.sqrt(self.vx**2 + self.vy**2)
        self.status.armed = self._armed
        self.status.mode = self._mode
    
    def get_status(self) -> DroneStatus:
        """Get current drone status telemetry."""
        return self.status
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (x, y, z) in local frame."""
        return self.x, self.y, self.z
    
    def get_target(self) -> Tuple[float, float, float]:
        """Get current target position (x, y, z) in local frame."""
        return self.target_x, self.target_y, self.target_z
    
    def at_target(self, threshold: float = 1.0) -> bool:
        """
        Check if drone is at its target position.
        
        Args:
            threshold: Distance threshold in meters
            
        Returns:
            True if within threshold of target
        """
        dx = self.x - self.target_x
        dy = self.y - self.target_y
        dz = self.z - self.target_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return dist < threshold
    
    def distance_to(self, other: "SimulatedDrone") -> float:
        """
        Calculate distance to another drone.
        
        Args:
            other: Another SimulatedDrone instance
            
        Returns:
            Distance in meters
        """
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def __repr__(self) -> str:
        return (
            f"SimulatedDrone(id={self.drone_id}, "
            f"state={self.state.value}, "
            f"pos=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
            f"target=({self.target_x:.1f}, {self.target_y:.1f}, {self.target_z:.1f}))"
        )
