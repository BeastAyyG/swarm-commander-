#!/usr/bin/env python3
"""
cli_commander.py — Swarm Commander CLI
=======================================
Text-based command-line interface for drone swarm coordination.
Runs a fully simulated swarm in a local 2-D + altitude coordinate frame —
no ArduPilot / SITL or pygame required.

Usage:
    python3 src/cli_commander.py [--drones N] [--formation NAME] [--spacing M]
    python3 src/cli_commander.py --batch commands.txt

Available commands:
    takeoff [alt]           Arm all drones and climb to altitude (default 20 m)
    land                    Land all drones at current XY position
    goto <x> <y> [alt]      Navigate swarm leader to waypoint; followers hold formation
    formation <name>        Change swarm formation (V, ARROW, CIRCLE, WALL, LINE, GRID)
    add [n]                 Add n more drones to the fleet (default 1)
    remove <id>             Remove a drone from the fleet by ID
    simulate [steps] [dt]   Advance the physics simulation (default 10 steps × 1 s)
    status                  Print current fleet status
    reset                   Reset swarm to home positions
    formations              List available formation patterns
    help                    Show help for a command
    exit / quit             Shut down the CLI
"""

import argparse
import cmd
import math
import sys
import os
import textwrap
from typing import List, Optional

# Allow running directly from the src/ dir or from the repo root
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from formations import FORMATIONS, FORMATION_KEYS, get_formation_slots, get_slot_gps


# ─────────────────────────────────────────────
#  Simulated drone model
# ─────────────────────────────────────────────

class DroneAgent:
    """
    Simulated drone in a local 2-D + altitude coordinate frame.

    Positions are in metres relative to the home origin (0, 0).
    The drone moves toward its target at constant speed each simulation step.
    """

    GROUNDED = "GROUNDED"
    AIRBORNE = "AIRBORNE"
    LANDING  = "LANDING"

    def __init__(self, drone_id: int, x: float = 0.0, y: float = 0.0):
        self.drone_id = drone_id
        self.x = x
        self.y = y
        self.alt = 0.0

        self.target_x   = x
        self.target_y   = y
        self.target_alt = 0.0

        self.h_speed = 10.0   # horizontal speed  (m/s)
        self.v_speed =  3.0   # vertical speed    (m/s)

        self.state = self.GROUNDED

    # ── physics step ──────────────────────────────────────
    def step(self, dt: float = 1.0):
        """Advance position toward target by one time step of dt seconds."""
        # Horizontal
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist_h = math.hypot(dx, dy)
        if dist_h > 0.01:
            move = min(self.h_speed * dt, dist_h)
            self.x += (dx / dist_h) * move
            self.y += (dy / dist_h) * move
        else:
            self.x = self.target_x
            self.y = self.target_y

        # Vertical
        dalt = self.target_alt - self.alt
        if abs(dalt) > 0.01:
            move_alt = min(self.v_speed * dt, abs(dalt))
            self.alt += math.copysign(move_alt, dalt)
        else:
            self.alt = self.target_alt

        # State transitions
        if self.state == self.LANDING and self.alt <= 0.01:
            self.alt  = 0.0
            self.state = self.GROUNDED

    # ── commands ──────────────────────────────────────────
    def arm_takeoff(self, target_alt: float = 20.0):
        self.target_alt = target_alt
        self.state = self.AIRBORNE

    def land(self):
        self.target_alt = 0.0
        self.state = self.LANDING

    def goto(self, x: float, y: float, alt: Optional[float] = None):
        self.target_x = x
        self.target_y = y
        if alt is not None:
            self.target_alt = alt

    # ── helpers ───────────────────────────────────────────
    @property
    def distance_to_target(self) -> float:
        return math.hypot(self.target_x - self.x,
                          self.target_y - self.y)

    @property
    def at_target(self) -> bool:
        return self.distance_to_target < 0.5 and abs(self.alt - self.target_alt) < 0.5

    def __repr__(self):
        return (f"Drone {self.drone_id:2d} | state={self.state:8s} | "
                f"pos=({self.x:7.1f}, {self.y:7.1f}, {self.alt:5.1f} m) | "
                f"target=({self.target_x:7.1f}, {self.target_y:7.1f}, {self.target_alt:5.1f} m)")


# ─────────────────────────────────────────────
#  Swarm fleet manager
# ─────────────────────────────────────────────

class SwarmFleet:
    """
    Manages a collection of DroneAgents in a formation-based swarm.

    The *leader slot* (index 0) anchors the formation.  All other drones
    are positioned relative to the leader using the formation offsets and
    the configurable spacing distance.
    """

    def __init__(self, num_drones: int = 3,
                 formation: str = "V",
                 spacing: float = 15.0):
        self.spacing    = spacing
        self.formation  = formation if formation in FORMATIONS else "V"
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.cruise_alt = 20.0
        self.heading    = 0.0   # radians (0 = North/+Y)

        # Spawn drones offset by initial formation slots
        self.drones: List[DroneAgent] = []
        self._next_id = 1
        for _ in range(num_drones):
            self._spawn_drone()

    # ── internal helpers ──────────────────────────────────
    def _spawn_drone(self):
        """Add one drone at its home slot position in the current formation.

        Uses the same body-frame → world-frame rotation as _assign_formation_targets
        so that spawned drones are consistent with formation targets.
        """
        did = self._next_id
        self._next_id += 1
        slots = get_formation_slots(self.formation, did)
        slot  = slots[did - 1] if (did - 1) < len(slots) else (-(did - 1), 0)
        fwd, right = slot
        # Rotate body-frame slot into world frame (East = x, North = y)
        d_n = (fwd * math.cos(self.heading) - right * math.sin(self.heading)) * self.spacing
        d_e = (fwd * math.sin(self.heading) + right * math.cos(self.heading)) * self.spacing
        d = DroneAgent(did,
                       x=self.waypoint_x + d_e,
                       y=self.waypoint_y + d_n)
        self.drones.append(d)

    def _assign_formation_targets(self):
        """Recompute each drone's (x, y) target from the current waypoint and formation.

        Slot coordinates are in the swarm's *body frame*:
          +fwd  = ahead of the leader (positive = forward along heading)
          +right = to the right of the leader

        They are rotated into the world frame (East / North) by the swarm heading:
          d_north = (fwd·cos(h) − right·sin(h)) × spacing
          d_east  = (fwd·sin(h) + right·cos(h)) × spacing
        """
        n = len(self.drones)
        slots = get_formation_slots(self.formation, n)
        for i, drone in enumerate(self.drones):
            fwd, right = slots[i] if i < len(slots) else (-(i), 0)
            # Rotate by swarm heading
            d_n = (fwd * math.cos(self.heading) - right * math.sin(self.heading)) * self.spacing
            d_e = (fwd * math.sin(self.heading) + right * math.cos(self.heading)) * self.spacing
            drone.goto(self.waypoint_x + d_e,
                       self.waypoint_y + d_n,
                       alt=self.cruise_alt if drone.state == DroneAgent.AIRBORNE else None)

    # ── fleet commands ────────────────────────────────────
    def takeoff(self, altitude: float = 20.0) -> List[str]:
        self.cruise_alt = altitude
        msgs = []
        for d in self.drones:
            if d.state == DroneAgent.GROUNDED:
                d.arm_takeoff(altitude)
                msgs.append(f"  D{d.drone_id}: GROUNDED → AIRBORNE ({altitude:.1f} m)")
            else:
                msgs.append(f"  D{d.drone_id}: already {d.state}")
        self._assign_formation_targets()
        return msgs

    def land(self) -> List[str]:
        msgs = []
        for d in self.drones:
            if d.state != DroneAgent.GROUNDED:
                prev = d.state
                d.land()
                msgs.append(f"  D{d.drone_id}: {prev} → LANDING")
            else:
                msgs.append(f"  D{d.drone_id}: already GROUNDED")
        return msgs

    def goto(self, x: float, y: float, alt: Optional[float] = None) -> List[str]:
        # Update swarm heading to face the new waypoint
        dx = x - self.waypoint_x
        dy = y - self.waypoint_y
        if math.hypot(dx, dy) > 0.1:
            self.heading = math.atan2(dx, dy)   # atan2(east, north)
        self.waypoint_x = x
        self.waypoint_y = y
        if alt is not None:
            self.cruise_alt = alt
        self._assign_formation_targets()
        airborne = [d for d in self.drones if d.state == DroneAgent.AIRBORNE]
        return [f"  Navigating {len(airborne)} airborne drones to "
                f"({x:.1f}, {y:.1f}) alt={self.cruise_alt:.1f} m"]

    def set_formation(self, name: str) -> str:
        name = name.upper()
        if name not in FORMATIONS:
            return f"Unknown formation '{name}'. Available: {', '.join(FORMATION_KEYS)}"
        old = self.formation
        self.formation = name
        self._assign_formation_targets()
        return f"Formation changed: {old} → {name} ({FORMATIONS[name]['name']})"

    def add_drones(self, n: int = 1) -> List[str]:
        msgs = []
        for _ in range(n):
            self._spawn_drone()
            msgs.append(f"  Added Drone {self._next_id - 1}")
        self._assign_formation_targets()
        return msgs

    def remove_drone(self, drone_id: int) -> str:
        for i, d in enumerate(self.drones):
            if d.drone_id == drone_id:
                self.drones.pop(i)
                return f"Removed Drone {drone_id}"
        return f"Drone {drone_id} not found"

    def reset(self) -> str:
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.heading    = 0.0
        for d in self.drones:
            d.state = DroneAgent.GROUNDED
            d.alt   = 0.0
            d.target_alt = 0.0
        self._assign_formation_targets()
        # Teleport to formation positions
        for d in self.drones:
            d.x = d.target_x
            d.y = d.target_y
        return f"Swarm reset to home. {len(self.drones)} drones at ground level."

    def step(self, dt: float = 1.0):
        """Advance all drones by dt seconds."""
        for d in self.drones:
            d.step(dt)

    def simulate(self, steps: int = 10, dt: float = 1.0) -> str:
        """Run n steps and return a progress summary."""
        for _ in range(steps):
            self.step(dt)
        at_target = sum(1 for d in self.drones if d.at_target)
        return (f"Simulated {steps} steps × {dt:.1f} s = {steps * dt:.0f} s total. "
                f"{at_target}/{len(self.drones)} drones at target.")

    def status(self) -> str:
        """Return a formatted status table."""
        lines = []
        w_id   =  4
        w_state = 9
        w_pos  = 22
        w_tgt  = 22
        header = (f"{'ID':>{w_id}}  {'State':<{w_state}}  "
                  f"{'Position (x, y, alt)':^{w_pos}}  "
                  f"{'Target (x, y, alt)':^{w_tgt}}  {'At?':>4}")
        sep = "─" * len(header)
        lines.append(sep)
        lines.append(header)
        lines.append(sep)
        for d in self.drones:
            pos = f"({d.x:6.1f}, {d.y:6.1f}, {d.alt:5.1f})"
            tgt = f"({d.target_x:6.1f}, {d.target_y:6.1f}, {d.target_alt:5.1f})"
            at  = "✓" if d.at_target else "…"
            lines.append(f"{'D'+str(d.drone_id):>{w_id}}  {d.state:<{w_state}}  "
                         f"{pos:^{w_pos}}  {tgt:^{w_tgt}}  {at:>4}")
        lines.append(sep)
        airborne = sum(1 for d in self.drones if d.state == DroneAgent.AIRBORNE)
        fi = FORMATIONS[self.formation]
        lines.append(f"Formation: {fi['icon']} {self.formation} ({fi['name']})  |  "
                     f"Spacing: {self.spacing:.1f} m  |  "
                     f"Airborne: {airborne}/{len(self.drones)}  |  "
                     f"Waypoint: ({self.waypoint_x:.1f}, {self.waypoint_y:.1f})")
        return "\n".join(lines)


# ─────────────────────────────────────────────
#  Interactive CLI
# ─────────────────────────────────────────────

BANNER = r"""
╔══════════════════════════════════════════════════╗
║        🚁  Swarm Commander CLI  v1.0  🚁         ║
║     Drone Swarm Coordination System              ║
║     Type 'help' for commands, 'exit' to quit     ║
╚══════════════════════════════════════════════════╝
"""

PROMPT = "swarm> "


class SwarmCLI(cmd.Cmd):
    """Interactive command-line interface for the Swarm Commander."""

    intro  = BANNER
    prompt = PROMPT

    def __init__(self, fleet: SwarmFleet):
        super().__init__()
        self.fleet = fleet

    # ── command: takeoff ──────────────────────────────────
    def do_takeoff(self, arg: str):
        """takeoff [altitude_m]
        Arm all grounded drones and climb to altitude (default 20 m)."""
        try:
            alt = float(arg.strip()) if arg.strip() else 20.0
        except ValueError:
            print("Usage: takeoff [altitude_m]   (e.g. takeoff 25)")
            return
        print(f"[CMD] Takeoff ordered — target altitude {alt:.1f} m")
        for msg in self.fleet.takeoff(alt):
            print(msg)

    # ── command: land ─────────────────────────────────────
    def do_land(self, arg: str):
        """land
        Land all airborne drones at their current XY positions."""
        print("[CMD] Landing all drones…")
        for msg in self.fleet.land():
            print(msg)

    # ── command: goto ─────────────────────────────────────
    def do_goto(self, arg: str):
        """goto <x> <y> [altitude_m]
        Navigate the swarm to a waypoint in local frame (metres).
        Drones must be airborne. The swarm heading rotates toward the waypoint."""
        parts = arg.split()
        if len(parts) < 2:
            print("Usage: goto <x> <y> [altitude_m]   (e.g. goto 100 50 30)")
            return
        try:
            x   = float(parts[0])
            y   = float(parts[1])
            alt = float(parts[2]) if len(parts) >= 3 else None
        except ValueError:
            print("Error: x, y, and optional altitude must be numbers.")
            return
        alt_str = f" alt={alt:.1f} m" if alt is not None else ""
        print(f"[CMD] Waypoint → ({x:.1f}, {y:.1f}){alt_str}")
        for msg in self.fleet.goto(x, y, alt):
            print(msg)

    # ── command: formation ────────────────────────────────
    def do_formation(self, arg: str):
        """formation <name>
        Change swarm formation. Available: V, ARROW, CIRCLE, WALL, LINE, GRID."""
        name = arg.strip().upper()
        if not name:
            print("Usage: formation <name>   (e.g. formation GRID)")
            print(f"Available: {', '.join(FORMATION_KEYS)}")
            return
        result = self.fleet.set_formation(name)
        print(f"[CMD] {result}")

    # ── command: formations ───────────────────────────────
    def do_formations(self, arg: str):
        """formations
        List all available formation patterns with descriptions."""
        print("\n  Available Formations:")
        print(f"  {'Key':<8}  {'Icon'}  {'Name':<12}  Description")
        print(f"  {'─'*8}  {'─'*4}  {'─'*12}  {'─'*40}")
        for key, info in FORMATIONS.items():
            active = " ◀ current" if key == self.fleet.formation else ""
            print(f"  {key:<8}  {info['icon']:>4}  {info['name']:<12}  "
                  f"{info['description']}{active}")
        print()

    # ── command: add ──────────────────────────────────────
    def do_add(self, arg: str):
        """add [n]
        Add n drones to the fleet (default 1)."""
        try:
            n = int(arg.strip()) if arg.strip() else 1
        except ValueError:
            print("Usage: add [n]   (e.g. add 2)")
            return
        if n < 1:
            print("n must be at least 1.")
            return
        for msg in self.fleet.add_drones(n):
            print(msg)
        print(f"[CMD] Fleet now has {len(self.fleet.drones)} drones.")

    # ── command: remove ───────────────────────────────────
    def do_remove(self, arg: str):
        """remove <drone_id>
        Remove a drone from the fleet by ID."""
        try:
            did = int(arg.strip())
        except (ValueError, AttributeError):
            print("Usage: remove <drone_id>   (e.g. remove 3)")
            return
        print(f"[CMD] {self.fleet.remove_drone(did)}")

    # ── command: simulate ─────────────────────────────────
    def do_simulate(self, arg: str):
        """simulate [steps] [dt]
        Advance the physics simulation.
        steps: number of steps (default 10)
        dt:    seconds per step (default 1.0)"""
        parts = arg.split()
        try:
            steps = int(parts[0])   if len(parts) >= 1 else 10
            dt    = float(parts[1]) if len(parts) >= 2 else 1.0
        except ValueError:
            print("Usage: simulate [steps] [dt]   (e.g. simulate 20 0.5)")
            return
        result = self.fleet.simulate(steps, dt)
        print(f"[SIM] {result}")

    # ── command: status ───────────────────────────────────
    def do_status(self, arg: str):
        """status
        Print current fleet status table."""
        print(self.fleet.status())

    # ── command: reset ────────────────────────────────────
    def do_reset(self, arg: str):
        """reset
        Return all drones to home positions and reset swarm state."""
        print(f"[CMD] {self.fleet.reset()}")

    # ── command: exit / quit ──────────────────────────────
    def do_exit(self, arg: str):
        """exit
        Shut down the Swarm Commander CLI."""
        print("Goodbye! Swarm Commander CLI shutting down.")
        return True

    def do_quit(self, arg: str):
        """quit
        Shut down the Swarm Commander CLI."""
        return self.do_exit(arg)

    def do_EOF(self, arg: str):
        """Handle Ctrl-D."""
        print()
        return self.do_exit(arg)

    # ── helpers ───────────────────────────────────────────
    def emptyline(self):
        """Do nothing on empty input line."""

    def default(self, line: str):
        print(f"Unknown command: '{line}'. Type 'help' for available commands.")


# ─────────────────────────────────────────────
#  Batch mode runner
# ─────────────────────────────────────────────

def run_batch(fleet: SwarmFleet, filepath: str):
    """Execute commands from a text file, one per line (# for comments)."""
    cli = SwarmCLI(fleet)
    try:
        with open(filepath) as fh:
            for lineno, raw in enumerate(fh, 1):
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                print(f"{PROMPT}{line}")
                stop = cli.onecmd(line)
                if stop:
                    break
    except FileNotFoundError:
        print(f"Batch file not found: {filepath}", file=sys.stderr)
        sys.exit(1)


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Swarm Commander — drone swarm coordination CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              python3 src/cli_commander.py
              python3 src/cli_commander.py --drones 5 --formation CIRCLE
              python3 src/cli_commander.py --batch commands.txt
        """),
    )
    parser.add_argument("--drones",    type=int,   default=3,
                        help="Initial number of drones (default: 3)")
    parser.add_argument("--formation", type=str,   default="V",
                        help="Initial formation: V, ARROW, CIRCLE, WALL, LINE, GRID (default: V)")
    parser.add_argument("--spacing",   type=float, default=15.0,
                        help="Formation spacing in metres (default: 15.0)")
    parser.add_argument("--batch",     type=str,   default=None,
                        help="Run commands from a file instead of interactively")
    args = parser.parse_args()

    formation = args.formation.upper()
    if formation not in FORMATIONS:
        print(f"Warning: unknown formation '{formation}', defaulting to V.")
        formation = "V"

    fleet = SwarmFleet(num_drones=args.drones,
                       formation=formation,
                       spacing=args.spacing)
    print(f"Swarm initialized: {args.drones} drones, "
          f"{FORMATIONS[formation]['name']} formation, "
          f"spacing={args.spacing:.1f} m")

    if args.batch:
        run_batch(fleet, args.batch)
    else:
        cli = SwarmCLI(fleet)
        try:
            cli.cmdloop()
        except KeyboardInterrupt:
            print("\nInterrupted. Goodbye!")


if __name__ == "__main__":
    main()
