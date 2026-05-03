#!/usr/bin/env python3
"""
config.py — Configuration Management

Centralized configuration for the swarm commander system.
"""

from dataclasses import dataclass, field
from typing import Dict, Any


@dataclass
class Config:
    """Global configuration constants."""
    
    # Earth model
    EARTH_RADIUS: float = 6378137.0  # meters (WGS84)
    
    # Default home location (Canberra, Australia - ArduPilot default)
    HOME_LAT: float = -35.363261
    HOME_LON: float = 149.165230
    HOME_ALT: float = 584.0  # meters AMSL
    
    # Formation defaults
    DEFAULT_SPACING: float = 15.0  # meters between drones
    DEFAULT_FORMATION: str = "V"
    DEFAULT_DRONES: int = 3
    MAX_DRONES: int = 7
    
    # Flight parameters
    DEFAULT_TAKEOFF_ALT: float = 20.0  # meters AGL
    CRUISE_ALT: float = 20.0  # meters AGL
    HORIZONTAL_SPEED: float = 10.0  # m/s
    VERTICAL_SPEED: float = 3.0  # m/s
    
    # Collision avoidance (APF)
    DRONE_AVOID_RADIUS: float = 14.0  # meters
    OBSTACLE_AVOID_RADIUS: float = 30.0  # meters
    DRONE_REPULSE_GAIN: float = 10.0
    OBSTACLE_REPULSE_GAIN: float = 20.0
    
    # SITL connection
    BASE_PORT: int = 5760  # First drone MAVLink port
    PORT_STEP: int = 10    # Port increment per drone
    BROADCAST_PORT: int = 5005  # UDP broadcast port
    
    # Timing
    SITL_STARTUP_WAIT: float = 15.0  # seconds for EKF initialization
    CONNECTION_TIMEOUT: float = 30.0  # seconds
    CONNECTION_RETRIES: int = 10
    CALIBRATION_WAIT: float = 15.0  # seconds for accel calibration
    
    # Visualization
    FPS: int = 60
    WINDOW_WIDTH: int = 1400
    WINDOW_HEIGHT: int = 900
    MIN_ZOOM: float = 0.08  # meters per pixel
    MAX_ZOOM: float = 2.0   # meters per pixel
    DEFAULT_MPP: float = 0.4  # meters per pixel


@dataclass
class SwarmConfig:
    """Runtime swarm configuration."""
    
    num_drones: int = 3
    formation: str = "V"
    spacing: float = 15.0
    cruise_alt: float = 20.0
    heading: float = 0.0  # radians, 0 = North
    
    # Waypoint tracking
    waypoint_x: float = 0.0
    waypoint_y: float = 0.0
    
    # Dynamic state
    active: bool = False
    mission_phase: str = "IDLE"  # IDLE, TAKEOFF, MISSION, RTL, LAND
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "num_drones": self.num_drones,
            "formation": self.formation,
            "spacing": self.spacing,
            "cruise_alt": self.cruise_alt,
            "heading": self.heading,
            "waypoint_x": self.waypoint_x,
            "waypoint_y": self.waypoint_y,
            "active": self.active,
            "mission_phase": self.mission_phase,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SwarmConfig":
        """Create from dictionary."""
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})


# Singleton instance for global config access
_global_config = Config()


def get_config() -> Config:
    """Get the global configuration singleton."""
    return _global_config
