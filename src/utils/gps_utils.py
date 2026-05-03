#!/usr/bin/env python3
"""
gps_utils.py — GPS Coordinate Conversion Utilities

Provides functions for converting between GPS coordinates (lat/lon/alt)
and local Cartesian coordinates (meters from home origin).
"""

import math
from dataclasses import dataclass
from typing import Tuple

from .config import get_config


@dataclass
class LocalCoord:
    """Local Cartesian coordinates (meters from home)."""
    x: float  # East
    y: float  # North
    z: float  # Up (altitude AGL)


def gps_to_local(lat: float, lon: float, alt: float = None) -> LocalCoord:
    """
    Convert GPS coordinates to local Cartesian coordinates.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters (optional, returns z=0 if not provided)
    
    Returns:
        LocalCoord with x (east), y (north), z (up) in meters from home
    """
    cfg = get_config()
    
    # Convert to radians
    lat_rad = math.radians(cfg.HOME_LAT)
    
    # Calculate offsets in meters
    dy = (lat - cfg.HOME_LAT) * cfg.EARTH_RADIUS * math.pi / 180.0
    dx = (lon - cfg.HOME_LON) * cfg.EARTH_RADIUS * math.cos(lat_rad) * math.pi / 180.0
    
    # Calculate altitude above home
    dz = 0.0
    if alt is not None:
        dz = alt - cfg.HOME_ALT
    
    return LocalCoord(x=dx, y=dy, z=dz)


def local_to_gps(x: float, y: float, z: float = None) -> Tuple[float, float, float]:
    """
    Convert local Cartesian coordinates to GPS coordinates.
    
    Args:
        x: East offset in meters from home
        y: North offset in meters from home
        z: Altitude in meters AGL (optional)
    
    Returns:
        Tuple of (latitude, longitude, altitude) in degrees/meters
    """
    cfg = get_config()
    
    # Convert to degrees
    dlat = y / cfg.EARTH_RADIUS * 180.0 / math.pi
    dlon = x / (cfg.EARTH_RADIUS * math.cos(math.radians(cfg.HOME_LAT))) * 180.0 / math.pi
    
    new_lat = cfg.HOME_LAT + dlat
    new_lon = cfg.HOME_LON + dlon
    new_alt = cfg.HOME_ALT + z if z is not None else cfg.HOME_ALT
    
    return new_lat, new_lon, new_alt


def offset_gps(
    base_lat: float,
    base_lon: float,
    base_alt: float,
    d_north: float,
    d_east: float
) -> Tuple[float, float, float]:
    """
    Calculate a new GPS position offset from a base position.
    
    Args:
        base_lat: Base latitude in degrees
        base_lon: Base longitude in degrees
        base_alt: Base altitude in meters AMSL
        d_north: Offset north in meters (positive = north)
        d_east: Offset east in meters (positive = east)
    
    Returns:
        Tuple of (new_lat, new_lon, new_alt)
    """
    cfg = get_config()
    
    # Convert offsets to degrees
    dlat = d_north / cfg.EARTH_RADIUS * 180.0 / math.pi
    dlon = d_east / (cfg.EARTH_RADIUS * math.cos(math.radians(base_lat))) * 180.0 / math.pi
    
    return base_lat + dlat, base_lon + dlon, base_alt


def rotate_offset(
    fwd: float,
    right: float,
    heading: float,
    spacing: float
) -> Tuple[float, float]:
    """
    Rotate a formation slot offset by the swarm heading.
    
    Converts body-frame offsets (forward/right) to world-frame offsets
    (north/east) using the swarm heading.
    
    Args:
        fwd: Forward offset in formation units (positive = ahead)
        right: Right offset in formation units (positive = right)
        heading: Swarm heading in radians (0 = North, π/2 = East)
        spacing: Distance in meters per formation unit
    
    Returns:
        Tuple of (d_north, d_east) in meters
    """
    d_north = (fwd * math.cos(heading) - right * math.sin(heading)) * spacing
    d_east = (fwd * math.sin(heading) + right * math.cos(heading)) * spacing
    return d_north, d_east


def calculate_heading(
    from_x: float,
    from_y: float,
    to_x: float,
    to_y: float
) -> float:
    """
    Calculate heading from one point to another.
    
    Args:
        from_x, from_y: Starting position (local coords, meters)
        to_x, to_y: Target position (local coords, meters)
    
    Returns:
        Heading in radians (0 = North, clockwise positive)
    """
    dx = to_x - from_x  # East
    dy = to_y - from_y  # North
    return math.atan2(dx, dy)


def distance_2d(
    x1: float,
    y1: float,
    x2: float,
    y2: float
) -> float:
    """
    Calculate 2D Euclidean distance between two points.
    
    Args:
        x1, y1: First point coordinates (meters)
        x2, y2: Second point coordinates (meters)
    
    Returns:
        Distance in meters
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def interpolate_position(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    t: float
) -> Tuple[float, float]:
    """
    Linearly interpolate between two positions.
    
    Args:
        start_x, start_y: Start position
        end_x, end_y: End position
        t: Interpolation factor (0 = start, 1 = end)
    
    Returns:
        Tuple of (x, y) at interpolation point
    """
    x = start_x + (end_x - start_x) * t
    y = start_y + (end_y - start_y) * t
    return x, y
