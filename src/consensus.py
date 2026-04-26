"""
consensus.py — Dynamic Role Reassignment via Consensus Algorithm
=================================================================
Implements a leader-election / role-reassignment protocol for the swarm.
When a drone loses battery (<20%) or fails, its role (scout/leader/relay)
is redistributed to eligible peers using a weighted-score consensus.

Roles
-----
- leader  : Head of the swarm; sets waypoints and broadcasts targets.
- scout   : Advance recon, flies ahead of formation.
- relay   : Communication relay, maintains radio link back to GCS.

Public API
----------
    consensus = SwarmConsensus(drone_ids)
    consensus.update_drone(drone_id, battery=85, alive=True)
    new_roles = consensus.get_roles()        # {drone_id: role}
    consensus.trigger_reassignment(failed_id)
"""

from __future__ import annotations

import threading
import time
import logging
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)

# Role priority order — sorted by operational criticality
ROLES = ["leader", "scout", "relay"]

# Minimum battery level to hold a role (%)
BATTERY_LOW_THRESHOLD = 20.0


class DroneState:
    """Runtime telemetry record for one drone in the consensus registry."""

    def __init__(self, drone_id: str) -> None:
        """
        Initialise a DroneState entry.

        Args:
            drone_id: Unique string identifier for the drone.
        """
        self.drone_id = drone_id
        self.battery: float = 100.0
        self.alive: bool = True
        self.role: Optional[str] = None
        self.last_seen: float = time.time()

    @property
    def eligible(self) -> bool:
        """Return True when the drone can accept a new role assignment."""
        return self.alive and self.battery >= BATTERY_LOW_THRESHOLD

    def score(self) -> float:
        """
        Compute a fitness score used during consensus election.

        Higher score → better candidate for critical roles.
        Score is based on battery level penalised when battery is low.

        Returns:
            Float fitness score in range [0, 100].
        """
        if not self.eligible:
            return 0.0
        return self.battery


class SwarmConsensus:
    """
    Thread-safe consensus manager for swarm role assignment.

    The manager maintains the current role map and handles automatic
    reassignment when a drone fails or drops below the battery threshold.

    Args:
        drone_ids: Initial list of drone identifier strings.
    """

    def __init__(self, drone_ids: List[str]) -> None:
        """
        Initialise the consensus engine.

        Args:
            drone_ids: Identifiers for each drone participating in the swarm.
        """
        self._lock = threading.Lock()
        self._drones: Dict[str, DroneState] = {
            did: DroneState(did) for did in drone_ids
        }
        self._roles: Dict[str, Optional[str]] = {did: None for did in drone_ids}
        self._assign_initial_roles()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_drone(
        self,
        drone_id: str,
        battery: float,
        alive: bool = True,
    ) -> bool:
        """
        Update telemetry for a drone and trigger reassignment if needed.

        Args:
            drone_id: Target drone identifier.
            battery:  Current battery level (0–100 %).
            alive:    False if the drone has crashed / gone silent.

        Returns:
            True if a role reassignment was triggered, False otherwise.
        """
        with self._lock:
            if drone_id not in self._drones:
                logger.warning("Unknown drone %s — ignoring update", drone_id)
                return False

            state = self._drones[drone_id]
            prev_eligible = state.eligible
            state.battery = float(battery)
            state.alive = alive
            state.last_seen = time.time()

            reassigned = False
            # Trigger reassignment when drone becomes ineligible
            if prev_eligible and not state.eligible:
                logger.info(
                    "Drone %s became ineligible (battery=%.1f%%, alive=%s). "
                    "Triggering consensus reassignment.",
                    drone_id,
                    battery,
                    alive,
                )
                self._reassign_role(drone_id)
                reassigned = True

            return reassigned

    def trigger_reassignment(self, failed_drone_id: str) -> Dict[str, Optional[str]]:
        """
        Manually trigger role reassignment for a failed drone.

        Useful when failure is detected externally (e.g. heartbeat timeout).

        Args:
            failed_drone_id: The drone whose role must be redistributed.

        Returns:
            Updated role map ``{drone_id: role}``.
        """
        with self._lock:
            if failed_drone_id in self._drones:
                self._drones[failed_drone_id].alive = False
            self._reassign_role(failed_drone_id)
            return dict(self._roles)

    def get_roles(self) -> Dict[str, Optional[str]]:
        """
        Return the current role map.

        Returns:
            ``{drone_id: role}`` snapshot (copy).
        """
        with self._lock:
            return dict(self._roles)

    def add_drone(self, drone_id: str) -> None:
        """
        Register a new drone with the consensus engine.

        The drone starts without a role; it will be assigned at the next
        reassignment cycle or when an existing drone fails.

        Args:
            drone_id: Unique identifier string for the new drone.
        """
        with self._lock:
            if drone_id not in self._drones:
                self._drones[drone_id] = DroneState(drone_id)
                self._roles[drone_id] = None
                logger.info("Drone %s added to consensus registry.", drone_id)

    def remove_drone(self, drone_id: str) -> None:
        """
        Remove a drone from the consensus registry (e.g. permanent failure).

        Its role is immediately redistributed.

        Args:
            drone_id: Drone to remove.
        """
        with self._lock:
            if drone_id in self._drones:
                role = self._roles.pop(drone_id, None)
                del self._drones[drone_id]
                logger.info(
                    "Drone %s removed. Role '%s' will be redistributed.", drone_id, role
                )
                if role:
                    self._elect_for_role(role, exclude=drone_id)

    def get_leader(self) -> Optional[str]:
        """
        Return the drone_id of the current leader.

        Returns:
            Drone id string or None if no leader is assigned.
        """
        with self._lock:
            return self._get_role_holder("leader")

    # ------------------------------------------------------------------
    # Internal helpers (must be called with self._lock held)
    # ------------------------------------------------------------------

    def _assign_initial_roles(self) -> None:
        """Distribute all defined roles across the initial drone set."""
        drones = list(self._drones.keys())
        for i, role in enumerate(ROLES):
            if i < len(drones):
                self._roles[drones[i]] = role

    def _reassign_role(self, failed_drone_id: str) -> None:
        """
        Redistribute the role held by *failed_drone_id* to the best candidate.

        Args:
            failed_drone_id: Drone whose role is being vacated.
        """
        vacated_role = self._roles.get(failed_drone_id)
        self._roles[failed_drone_id] = None

        if vacated_role:
            new_holder = self._elect_for_role(vacated_role, exclude=failed_drone_id)
            if new_holder:
                logger.info(
                    "Role '%s' reassigned from %s → %s",
                    vacated_role,
                    failed_drone_id,
                    new_holder,
                )
            else:
                logger.warning(
                    "No eligible drone available to take over role '%s'.", vacated_role
                )

    def _elect_for_role(self, role: str, exclude: str = "") -> Optional[str]:
        """
        Elect the best available drone for the given role.

        Selection criterion: highest battery score among eligible drones
        that don't already hold a higher-priority role.

        Args:
            role:    Target role to assign.
            exclude: Drone id to skip (the failed drone).

        Returns:
            Drone id of the elected drone, or None if no candidate found.
        """
        candidates = [
            (did, state)
            for did, state in self._drones.items()
            if did != exclude
            and state.eligible
            and self._roles.get(did) is None
        ]
        if not candidates:
            return None

        # Pick candidate with highest fitness score
        best = max(candidates, key=lambda x: x[1].score())
        elected_id = best[0]
        self._roles[elected_id] = role
        return elected_id

    def _get_role_holder(self, role: str) -> Optional[str]:
        """Return the drone holding *role*, or None."""
        for did, r in self._roles.items():
            if r == role:
                return did
        return None
