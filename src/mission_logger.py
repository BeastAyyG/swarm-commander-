"""
mission_logger.py — SQLite Mission Decision Logger
===================================================
Records every significant swarm event to an SQLite database so the full
mission history can be replayed and analysed after landing.

Logged event types
------------------
- role_change    : A drone's role was reassigned.
- path_replan    : APF replanned a drone's path (obstacle or avoidance).
- formation_switch: Swarm transitioned to a new formation.
- telemetry      : Periodic position/battery snapshot.
- custom         : Any caller-defined message.

Usage
-----
    log = MissionLogger("mission_001.db")
    log.log_role_change(drone_id="1", old_role="scout", new_role="leader", reason="battery_low")
    log.log_formation_switch(from_formation="V", to_formation="CIRCLE")
    log.close()
"""

from __future__ import annotations

import sqlite3
import threading
import time
import json
import logging
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

_CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS swarm_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp   REAL    NOT NULL,
    event_type  TEXT    NOT NULL,
    drone_id    TEXT,
    data        TEXT
);
"""

_CREATE_INDEX_SQL = """
CREATE INDEX IF NOT EXISTS idx_swarm_events_type
    ON swarm_events (event_type, timestamp);
"""


class MissionLogger:
    """
    Thread-safe SQLite logger for swarm mission events.

    Args:
        db_path: Path to the SQLite file.  Defaults to ``swarm_mission.db``
                 in the current working directory.
    """

    def __init__(self, db_path: str = "swarm_mission.db") -> None:
        """
        Open (or create) the SQLite database.

        Args:
            db_path: File-system path for the SQLite database file.
        """
        self._db_path = db_path
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute(_CREATE_TABLE_SQL)
            self._conn.execute(_CREATE_INDEX_SQL)
            self._conn.commit()
        logger.info("MissionLogger opened database: %s", db_path)

    # ------------------------------------------------------------------
    # Convenience log helpers
    # ------------------------------------------------------------------

    def log_role_change(
        self,
        drone_id: str,
        old_role: Optional[str],
        new_role: Optional[str],
        reason: str = "",
    ) -> None:
        """
        Record a drone role reassignment.

        Args:
            drone_id: Affected drone.
            old_role: Previous role (may be None if first assignment).
            new_role: Newly assigned role.
            reason:   Human-readable reason for the change.
        """
        self._log(
            event_type="role_change",
            drone_id=drone_id,
            data={"old_role": old_role, "new_role": new_role, "reason": reason},
        )

    def log_path_replan(
        self,
        drone_id: str,
        trigger: str,
        old_target: Optional[Dict],
        new_target: Optional[Dict],
    ) -> None:
        """
        Record an APF path replan event.

        Args:
            drone_id:   Drone that replanned.
            trigger:    What caused the replan ('moving_obstacle', 'drone_proximity', …).
            old_target: Previous target coordinates dict or None.
            new_target: New target coordinates dict or None.
        """
        self._log(
            event_type="path_replan",
            drone_id=drone_id,
            data={
                "trigger": trigger,
                "old_target": old_target,
                "new_target": new_target,
            },
        )

    def log_formation_switch(
        self,
        from_formation: Optional[str],
        to_formation: str,
        commander_id: Optional[str] = None,
    ) -> None:
        """
        Record a formation transition event.

        Args:
            from_formation: Name of the previous formation (None if first assignment).
            to_formation:   Name of the new formation.
            commander_id:   Drone (or GCS) that issued the morph command.
        """
        self._log(
            event_type="formation_switch",
            drone_id=commander_id,
            data={
                "from_formation": from_formation,
                "to_formation": to_formation,
            },
        )

    def log_telemetry(
        self,
        drone_id: str,
        lat: float,
        lon: float,
        alt: float,
        battery: float,
        role: Optional[str],
        formation: Optional[str],
    ) -> None:
        """
        Record a telemetry snapshot for one drone.

        Args:
            drone_id:  Drone identifier.
            lat:       GPS latitude.
            lon:       GPS longitude.
            alt:       Altitude above home (metres).
            battery:   Battery level (0–100 %).
            role:      Current role string.
            formation: Current formation name.
        """
        self._log(
            event_type="telemetry",
            drone_id=drone_id,
            data={
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "battery": battery,
                "role": role,
                "formation": formation,
            },
        )

    def log_custom(
        self,
        message: str,
        drone_id: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Record a free-form custom event.

        Args:
            message:  Description of the event.
            drone_id: Associated drone (optional).
            extra:    Additional key-value data to persist.
        """
        payload: Dict[str, Any] = {"message": message}
        if extra:
            payload.update(extra)
        self._log(event_type="custom", drone_id=drone_id, data=payload)

    # ------------------------------------------------------------------
    # Query helpers
    # ------------------------------------------------------------------

    def get_events(
        self,
        event_type: Optional[str] = None,
        drone_id: Optional[str] = None,
        since: Optional[float] = None,
        limit: int = 1000,
    ) -> list:
        """
        Query logged events.

        Args:
            event_type: Filter by event type (None = all types).
            drone_id:   Filter by drone (None = all drones).
            since:      Unix timestamp lower bound (None = no lower bound).
            limit:      Maximum number of rows to return.

        Returns:
            List of dicts with keys: id, timestamp, event_type, drone_id, data.
        """
        clauses = []
        params: list = []
        if event_type:
            clauses.append("event_type = ?")
            params.append(event_type)
        if drone_id:
            clauses.append("drone_id = ?")
            params.append(drone_id)
        if since is not None:
            clauses.append("timestamp >= ?")
            params.append(since)

        where = ("WHERE " + " AND ".join(clauses)) if clauses else ""
        sql = f"SELECT * FROM swarm_events {where} ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)

        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()

        return [
            {
                "id": r["id"],
                "timestamp": r["timestamp"],
                "event_type": r["event_type"],
                "drone_id": r["drone_id"],
                "data": json.loads(r["data"]) if r["data"] else {},
            }
            for r in rows
        ]

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Flush and close the database connection."""
        with self._lock:
            self._conn.commit()
            self._conn.close()
        logger.info("MissionLogger closed.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _log(
        self,
        event_type: str,
        drone_id: Optional[str],
        data: Optional[Dict],
    ) -> None:
        """
        Insert a single event row.

        Args:
            event_type: Event type string.
            drone_id:   Associated drone id or None.
            data:       Payload dict (will be JSON-serialised).
        """
        ts = time.time()
        data_json = json.dumps(data) if data else None
        with self._lock:
            self._conn.execute(
                "INSERT INTO swarm_events (timestamp, event_type, drone_id, data) "
                "VALUES (?, ?, ?, ?)",
                (ts, event_type, drone_id, data_json),
            )
            self._conn.commit()
