#!/usr/bin/env python3
"""
Swarm Commander — Drone Agents Package

This package provides drone agent implementations:
- simulated_drone: Simulated drone model for testing
- sitl_drone: ArduPilot SITL integration
- swarm_agent: UDP-based swarm agent for distributed control
"""

from .simulated_drone import SimulatedDrone

__all__ = ["SimulatedDrone"]
