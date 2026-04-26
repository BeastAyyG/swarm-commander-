"""
monitor.py — Web-Based Swarm Monitor (Flask + SocketIO)
========================================================
Real-time dashboard that shows each drone's:
  - GPS position (lat / lon / alt)
  - Battery percentage
  - Role (scout / leader / relay)
  - Formation name and slot status

Updates are pushed to connected browsers via WebSocket every 500 ms.

Running the monitor
-------------------
    python3 -m src.monitor          # default port 5000
    python3 -m src.monitor --port 8080

Integrating with swarm code
----------------------------
    monitor = SwarmMonitor()
    monitor.update_drone("1", lat=-35.363, lon=149.165, alt=20.0,
                         battery=85.0, role="leader", formation="V",
                         formation_ok=True)
    monitor.run(debug=False)        # starts Flask in a background thread

The monitor also exposes :meth:`update_drone` which is safe to call from
any thread and works even when no browser is connected.
"""

from __future__ import annotations

import os
import threading
import time
import json
import logging
from typing import Dict, Any, Optional

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

logger = logging.getLogger(__name__)

# Broadcast interval in seconds
PUSH_INTERVAL = 0.5

_TEMPLATE_FOLDER = os.path.join(os.path.dirname(__file__), "templates")


class SwarmMonitor:
    """
    Flask + SocketIO real-time swarm dashboard.

    Args:
        host:  Bind address (default ``'0.0.0.0'``).
        port:  TCP port (default ``5000``).
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5000) -> None:
        """
        Initialise the web monitor.

        Args:
            host: Network interface to bind the Flask server to.
            port: TCP port to listen on.
        """
        self.host = host
        self.port = port

        self.app = Flask(
            __name__,
            template_folder=_TEMPLATE_FOLDER,
        )
        self.app.config["SECRET_KEY"] = "swarm-monitor-secret"
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode="threading",
        )

        self._lock = threading.Lock()
        self._drones: Dict[str, Dict[str, Any]] = {}
        self._formation: Optional[str] = None
        self._mission_events: list = []

        self._register_routes()
        self._push_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_drone(
        self,
        drone_id: str,
        lat: float = 0.0,
        lon: float = 0.0,
        alt: float = 0.0,
        battery: float = 100.0,
        role: Optional[str] = None,
        formation: Optional[str] = None,
        formation_ok: bool = True,
    ) -> None:
        """
        Update telemetry for a single drone.

        This method is thread-safe and can be called from any thread.

        Args:
            drone_id:     Unique drone identifier string.
            lat:          GPS latitude.
            lon:          GPS longitude.
            alt:          Altitude above home (metres).
            battery:      Battery level (0–100 %).
            role:         Current role string ('leader', 'scout', 'relay').
            formation:    Active formation name.
            formation_ok: True if drone is in its correct formation slot.
        """
        with self._lock:
            self._drones[drone_id] = {
                "id": drone_id,
                "lat": lat,
                "lon": lon,
                "alt": round(alt, 2),
                "battery": round(battery, 1),
                "role": role,
                "formation": formation,
                "formation_ok": formation_ok,
                "ts": time.time(),
            }
            if formation:
                self._formation = formation

    def push_event(self, event: Dict[str, Any]) -> None:
        """
        Append a mission event to the live event feed.

        Args:
            event: Arbitrary dict describing the event.  A ``timestamp``
                   key is added automatically.
        """
        with self._lock:
            event.setdefault("timestamp", time.time())
            self._mission_events.append(event)
            # Keep at most 200 events in memory
            if len(self._mission_events) > 200:
                self._mission_events = self._mission_events[-200:]

    def run(self, debug: bool = False, background: bool = True) -> None:
        """
        Start the Flask/SocketIO server.

        Args:
            debug:      Enable Flask debug mode.
            background: If True, starts the server in a daemon thread so
                        it does not block the calling thread.
        """
        self._start_push_thread()
        if background:
            t = threading.Thread(
                target=self.socketio.run,
                kwargs=dict(
                    app=self.app,
                    host=self.host,
                    port=self.port,
                    debug=debug,
                    use_reloader=False,
                    log_output=False,
                ),
                daemon=True,
            )
            t.start()
            logger.info("Swarm monitor started on http://%s:%d", self.host, self.port)
        else:
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                debug=debug,
                use_reloader=False,
            )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _register_routes(self) -> None:
        """Attach Flask routes and SocketIO event handlers."""

        @self.app.route("/")
        def index():
            """Serve the dashboard HTML."""
            return render_template("dashboard.html")

        @self.app.route("/api/drones")
        def api_drones():
            """REST endpoint — current drone telemetry snapshot."""
            with self._lock:
                return jsonify(list(self._drones.values()))

        @self.app.route("/api/events")
        def api_events():
            """REST endpoint — recent mission events."""
            with self._lock:
                return jsonify(self._mission_events[-50:])

        @self.socketio.on("connect")
        def on_connect():
            """Send full state snapshot on new client connection."""
            with self._lock:
                payload = {
                    "drones": list(self._drones.values()),
                    "formation": self._formation,
                    "events": self._mission_events[-20:],
                }
            emit("snapshot", payload)

        @self.socketio.on("request_snapshot")
        def on_snapshot(_data=None):
            """Client-requested snapshot."""
            with self._lock:
                payload = {
                    "drones": list(self._drones.values()),
                    "formation": self._formation,
                    "events": self._mission_events[-20:],
                }
            emit("snapshot", payload)

    def _start_push_thread(self) -> None:
        """Start the background telemetry push thread."""
        if self._push_thread and self._push_thread.is_alive():
            return

        def _push_loop():
            while True:
                time.sleep(PUSH_INTERVAL)
                with self._lock:
                    payload = {
                        "drones": list(self._drones.values()),
                        "formation": self._formation,
                    }
                try:
                    self.socketio.emit("telemetry_update", payload)
                except Exception:
                    pass

        self._push_thread = threading.Thread(target=_push_loop, daemon=True)
        self._push_thread.start()


# Allow running standalone for testing
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Swarm Monitor")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    monitor = SwarmMonitor(host=args.host, port=args.port)

    # Inject fake telemetry for demo purposes
    import random

    def _fake_telemetry():
        roles = ["leader", "scout", "relay"]
        formations = ["V", "CIRCLE", "LINE", "GRID"]
        drones = [str(i) for i in range(1, 6)]
        while True:
            for did in drones:
                monitor.update_drone(
                    did,
                    lat=-35.363261 + random.uniform(-0.001, 0.001),
                    lon=149.165230 + random.uniform(-0.001, 0.001),
                    alt=random.uniform(18, 25),
                    battery=random.uniform(60, 100),
                    role=random.choice(roles),
                    formation=random.choice(formations),
                    formation_ok=random.random() > 0.1,
                )
            time.sleep(0.5)

    t = threading.Thread(target=_fake_telemetry, daemon=True)
    t.start()

    print(f"Dashboard → http://{args.host}:{args.port}")
    monitor.run(background=False)
