#!/usr/bin/env python3
"""
test_swarm_core.py — Unit tests for core swarm logic
=====================================================
Tests the formation calculations, APF avoidance engine,
drone simulation model, and CLI swarm fleet manager.

Run with:
    pytest tests/test_swarm_core.py -v
or:
    python3 -m pytest tests/test_swarm_core.py -v
"""

import math
import sys
import os
import unittest

# ── path setup ────────────────────────────────────────────
# Support running from repo root or from the tests/ directory
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SRC_DIR   = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from formations import (
    FORMATIONS, FORMATION_KEYS,
    get_formation_slots, get_slot_gps,
)
from avoidance import APFEngine
from cli_commander import DroneAgent, SwarmFleet


# ══════════════════════════════════════════════════════════
#  1.  Formation tests
# ══════════════════════════════════════════════════════════

class TestFormations(unittest.TestCase):
    """Tests for formations.py — blueprints and GPS slot calculations."""

    # ── blueprint sanity ──────────────────────────────────
    def test_all_formation_keys_present(self):
        """Every expected formation key must be in FORMATIONS."""
        for key in ("V", "ARROW", "CIRCLE", "WALL", "LINE", "GRID"):
            self.assertIn(key, FORMATIONS, f"Missing formation: {key}")

    def test_each_formation_has_required_fields(self):
        for key, info in FORMATIONS.items():
            self.assertIn("name",        info, f"{key} missing 'name'")
            self.assertIn("icon",        info, f"{key} missing 'icon'")
            self.assertIn("description", info, f"{key} missing 'description'")
            self.assertIn("slots",       info, f"{key} missing 'slots'")
            self.assertIsInstance(info["slots"], list, f"{key} slots must be a list")
            self.assertGreater(len(info["slots"]), 0, f"{key} slots must be non-empty")

    def test_formation_slots_are_tuples_of_two_numbers(self):
        for key, info in FORMATIONS.items():
            for i, slot in enumerate(info["slots"]):
                self.assertEqual(len(slot), 2,
                                 f"{key} slot {i} should be a 2-tuple")
                for val in slot:
                    self.assertIsInstance(val, (int, float),
                                          f"{key} slot {i} values must be numeric")

    def test_grid_formation_has_at_least_seven_slots(self):
        # GRID is a proper 3×3 pattern (9 positions), so it supports up to 9 drones.
        self.assertEqual(len(FORMATIONS["GRID"]["slots"]), 9)

    # ── get_formation_slots ───────────────────────────────
    def test_get_slots_truncates_to_num_drones(self):
        for key in FORMATION_KEYS:
            slots = get_formation_slots(key, 3)
            self.assertEqual(len(slots), 3,
                             f"{key}: expected 3 slots, got {len(slots)}")

    def test_get_slots_unknown_formation_falls_back_to_V(self):
        slots_unknown = get_formation_slots("NONEXISTENT", 2)
        slots_v       = get_formation_slots("V", 2)
        self.assertEqual(slots_unknown, slots_v)

    def test_get_slots_single_drone(self):
        for key in FORMATION_KEYS:
            slots = get_formation_slots(key, 1)
            self.assertEqual(len(slots), 1)

    # ── get_slot_gps ──────────────────────────────────────
    def test_slot_gps_leader_at_origin(self):
        """Leader slot (0, 0) with zero heading must equal the base GPS."""
        base_lat, base_lon = -35.363261, 149.165230
        lat, lon = get_slot_gps(base_lat, base_lon,
                                heading_rad=0.0,
                                slot_fwd=0, slot_right=0,
                                spacing_m=15.0)
        self.assertAlmostEqual(lat, base_lat, places=7)
        self.assertAlmostEqual(lon, base_lon, places=7)

    def test_slot_gps_northward_offset(self):
        """A forward offset (fwd=1, right=0) with heading=0 should move north."""
        base_lat, base_lon = -35.363261, 149.165230
        lat, lon = get_slot_gps(base_lat, base_lon,
                                heading_rad=0.0,
                                slot_fwd=1, slot_right=0,
                                spacing_m=100.0)
        self.assertGreater(lat, base_lat,  "Should move north (lat increases)")
        self.assertAlmostEqual(lon, base_lon, places=5)

    def test_slot_gps_eastward_offset(self):
        """A right offset (fwd=0, right=1) with heading=0 should move east."""
        base_lat, base_lon = -35.363261, 149.165230
        lat, lon = get_slot_gps(base_lat, base_lon,
                                heading_rad=0.0,
                                slot_fwd=0, slot_right=1,
                                spacing_m=100.0)
        self.assertAlmostEqual(lat, base_lat, places=5)
        self.assertGreater(lon, base_lon, "Should move east (lon increases)")

    def test_slot_gps_symmetry(self):
        """Left and right slots equidistant from centre should mirror each other."""
        base_lat, base_lon = 0.0, 0.0
        lat_l, lon_l = get_slot_gps(base_lat, base_lon, 0.0, 0,  1, 200.0)
        lat_r, lon_r = get_slot_gps(base_lat, base_lon, 0.0, 0, -1, 200.0)
        self.assertAlmostEqual(lat_l, lat_r, places=6)
        self.assertAlmostEqual(lon_l + lon_r, 2 * base_lon, places=6)


# ══════════════════════════════════════════════════════════
#  2.  APF Avoidance Engine tests
# ══════════════════════════════════════════════════════════

class TestAPFEngine(unittest.TestCase):
    """Tests for avoidance.py — Artificial Potential Field engine."""

    def setUp(self):
        self.apf = APFEngine(drone_avoid_radius=14.0,
                             obstacle_avoid_radius=30.0,
                             drone_repulse_gain=10.0,
                             obstacle_repulse_gain=20.0)

    # ── attractive force ──────────────────────────────────
    def test_attractive_zero_at_target(self):
        fx, fy = self.apf.compute_attractive(5.0, 5.0, 5.0, 5.0)
        self.assertAlmostEqual(fx, 0.0)
        self.assertAlmostEqual(fy, 0.0)

    def test_attractive_direction(self):
        """Force must point toward the target."""
        fx, fy = self.apf.compute_attractive(0.0, 0.0, 10.0, 0.0)
        self.assertGreater(fx, 0.0)
        self.assertAlmostEqual(fy, 0.0, places=5)

    def test_attractive_symmetric_axes(self):
        fx_pos, _ = self.apf.compute_attractive(0, 0, 5, 0)
        fx_neg, _ = self.apf.compute_attractive(0, 0, -5, 0)
        self.assertAlmostEqual(fx_pos, -fx_neg, places=5)

    # ── drone repulsion ───────────────────────────────────
    def test_repulsion_not_active_beyond_radius(self):
        """No repulsion when neighbour is farther than avoid_radius."""
        _, _, active = self.apf.compute_drone_repulsion(
            0.0, 0.0, [(20.0, 0.0)])      # 20 m > avoid_radius 14 m
        self.assertFalse(active)

    def test_repulsion_active_within_radius(self):
        _, _, active = self.apf.compute_drone_repulsion(
            0.0, 0.0, [(5.0, 0.0)])       # 5 m < 14 m
        self.assertTrue(active)

    def test_repulsion_direction(self):
        """Repulsion from drone at +x should push current drone toward −x."""
        fx, _, _ = self.apf.compute_drone_repulsion(
            0.0, 0.0, [(5.0, 0.0)])
        self.assertLess(fx, 0.0, "Should be repelled away from neighbour at +x")

    def test_repulsion_no_neighbours(self):
        fx, fy, active = self.apf.compute_drone_repulsion(0.0, 0.0, [])
        self.assertAlmostEqual(fx, 0.0)
        self.assertAlmostEqual(fy, 0.0)
        self.assertFalse(active)

    # ── obstacle repulsion ────────────────────────────────
    def test_obstacle_not_active_beyond_radius(self):
        _, _, active = self.apf.compute_obstacle_repulsion(
            0.0, 0.0, [(40.0, 0.0, 5.0)])  # edge 35 m away > 30 m radius
        self.assertFalse(active)

    def test_obstacle_active_within_radius(self):
        _, _, active = self.apf.compute_obstacle_repulsion(
            0.0, 0.0, [(10.0, 0.0, 2.0)])  # edge ~8 m away < 30 m radius
        self.assertTrue(active)

    def test_obstacle_no_obstacles(self):
        fx, fy, active = self.apf.compute_obstacle_repulsion(0.0, 0.0, [])
        self.assertAlmostEqual(fx, 0.0)
        self.assertAlmostEqual(fy, 0.0)
        self.assertFalse(active)

    # ── total force ───────────────────────────────────────
    def test_total_force_no_obstacles_no_neighbours(self):
        """With no obstacles/neighbours, total force = attractive force."""
        fx_total, fy_total, _ = self.apf.compute_total_force(
            0.0, 0.0, 10.0, 0.0, [], [])
        fx_att, fy_att = self.apf.compute_attractive(0.0, 0.0, 10.0, 0.0)
        self.assertAlmostEqual(fx_total, 0.0 + fx_att, places=5)
        self.assertAlmostEqual(fy_total, 0.0 + fy_att, places=5)


# ══════════════════════════════════════════════════════════
#  3.  DroneAgent simulation model tests
# ══════════════════════════════════════════════════════════

class TestDroneAgent(unittest.TestCase):
    """Tests for DroneAgent — simulated single-drone physics."""

    def test_initial_state_is_grounded(self):
        d = DroneAgent(1)
        self.assertEqual(d.state, DroneAgent.GROUNDED)
        self.assertAlmostEqual(d.alt, 0.0)

    def test_takeoff_changes_state(self):
        d = DroneAgent(1)
        d.arm_takeoff(20.0)
        self.assertEqual(d.state, DroneAgent.AIRBORNE)
        self.assertAlmostEqual(d.target_alt, 20.0)

    def test_step_increases_altitude_after_takeoff(self):
        d = DroneAgent(1)
        d.arm_takeoff(20.0)
        d.step(1.0)
        self.assertGreater(d.alt, 0.0)

    def test_drone_reaches_target_altitude(self):
        d = DroneAgent(1)
        d.arm_takeoff(10.0)
        for _ in range(100):
            d.step(1.0)
        self.assertAlmostEqual(d.alt, 10.0, places=1)

    def test_drone_moves_horizontally(self):
        d = DroneAgent(1, x=0.0, y=0.0)
        d.arm_takeoff(5.0)
        d.goto(50.0, 0.0)
        for _ in range(100):
            d.step(1.0)
        self.assertAlmostEqual(d.x, 50.0, places=1)
        self.assertAlmostEqual(d.y,  0.0, places=1)

    def test_land_transitions_to_grounded(self):
        d = DroneAgent(1)
        d.arm_takeoff(10.0)
        for _ in range(20):   # reach altitude
            d.step(1.0)
        d.land()
        self.assertEqual(d.state, DroneAgent.LANDING)
        for _ in range(20):   # descend
            d.step(1.0)
        self.assertEqual(d.state, DroneAgent.GROUNDED)
        self.assertAlmostEqual(d.alt, 0.0, places=1)

    def test_at_target_true_when_on_target(self):
        d = DroneAgent(1, x=0.0, y=0.0)
        d.arm_takeoff(10.0)
        for _ in range(100):
            d.step(1.0)
        self.assertTrue(d.at_target)

    def test_at_target_false_when_far(self):
        d = DroneAgent(1, x=0.0, y=0.0)
        d.arm_takeoff(10.0)
        d.goto(200.0, 200.0)
        # After a single step, drone is still far from the 200-m target
        d.step(1.0)
        self.assertFalse(d.at_target)


# ══════════════════════════════════════════════════════════
#  4.  SwarmFleet tests
# ══════════════════════════════════════════════════════════

class TestSwarmFleet(unittest.TestCase):
    """Tests for SwarmFleet — high-level swarm coordination."""

    def _make_fleet(self, n=3, formation="V", spacing=15.0):
        return SwarmFleet(num_drones=n, formation=formation, spacing=spacing)

    def test_fleet_initialises_correct_drone_count(self):
        fleet = self._make_fleet(4)
        self.assertEqual(len(fleet.drones), 4)

    def test_fleet_drone_ids_are_unique(self):
        fleet = self._make_fleet(5)
        ids = [d.drone_id for d in fleet.drones]
        self.assertEqual(len(ids), len(set(ids)))

    def test_takeoff_sets_all_drones_airborne(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        for d in fleet.drones:
            self.assertEqual(d.state, DroneAgent.AIRBORNE)

    def test_land_sets_all_drones_landing(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        fleet.land()
        for d in fleet.drones:
            self.assertIn(d.state, (DroneAgent.LANDING, DroneAgent.GROUNDED))

    def test_set_formation_v(self):
        fleet = self._make_fleet(3)
        result = fleet.set_formation("V")
        self.assertEqual(fleet.formation, "V")
        self.assertIn("V", result)

    def test_set_formation_grid(self):
        fleet = self._make_fleet(3)
        result = fleet.set_formation("GRID")
        self.assertEqual(fleet.formation, "GRID")
        self.assertIn("GRID", result)

    def test_set_formation_circle(self):
        fleet = self._make_fleet(5)
        fleet.set_formation("CIRCLE")
        self.assertEqual(fleet.formation, "CIRCLE")

    def test_set_formation_invalid_returns_error(self):
        fleet = self._make_fleet(3)
        result = fleet.set_formation("TRIANGLE")
        self.assertIn("Unknown", result)
        self.assertEqual(fleet.formation, "V")   # unchanged

    def test_goto_updates_waypoint(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        fleet.goto(100.0, 50.0)
        self.assertAlmostEqual(fleet.waypoint_x, 100.0)
        self.assertAlmostEqual(fleet.waypoint_y, 50.0)

    def test_goto_with_altitude_updates_cruise_alt(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        fleet.goto(50.0, 50.0, alt=35.0)
        self.assertAlmostEqual(fleet.cruise_alt, 35.0)

    def test_add_drones_increases_fleet_size(self):
        fleet = self._make_fleet(2)
        fleet.add_drones(3)
        self.assertEqual(len(fleet.drones), 5)

    def test_remove_drone_decreases_fleet_size(self):
        fleet = self._make_fleet(3)
        first_id = fleet.drones[0].drone_id
        fleet.remove_drone(first_id)
        self.assertEqual(len(fleet.drones), 2)

    def test_remove_nonexistent_drone(self):
        fleet = self._make_fleet(2)
        result = fleet.remove_drone(999)
        self.assertIn("not found", result)

    def test_simulate_steps(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        fleet.goto(100.0, 0.0)
        result = fleet.simulate(steps=10, dt=1.0)
        self.assertIn("10 steps", result)
        # After 10 s at 10 m/s, drones should have moved toward target
        for d in fleet.drones:
            self.assertGreater(d.x, 0.0)   # moving in +x direction

    def test_reset_returns_drones_to_ground(self):
        fleet = self._make_fleet(3)
        fleet.takeoff(20.0)
        fleet.simulate(5)
        fleet.reset()
        for d in fleet.drones:
            self.assertEqual(d.state, DroneAgent.GROUNDED)
            self.assertAlmostEqual(d.alt, 0.0)

    def test_status_returns_string(self):
        fleet = self._make_fleet(3)
        status = fleet.status()
        self.assertIsInstance(status, str)
        self.assertIn("D1", status)
        self.assertIn("GROUNDED", status)

    def test_formation_drones_have_different_positions(self):
        """In a formation, all drones should have distinct slot targets."""
        fleet = self._make_fleet(5, formation="V", spacing=15.0)
        targets = [(d.target_x, d.target_y) for d in fleet.drones]
        # All targets should be unique
        self.assertEqual(len(targets), len(set(targets)))

    def test_grid_formation_positions(self):
        """GRID formation should spread drones across a 2-D grid."""
        fleet = self._make_fleet(7, formation="GRID", spacing=20.0)
        xs = [d.target_x for d in fleet.drones]
        ys = [d.target_y for d in fleet.drones]
        # Should have at least 2 distinct x-values and 2 distinct y-values
        self.assertGreater(len(set(round(x, 1) for x in xs)), 1)
        self.assertGreater(len(set(round(y, 1) for y in ys)), 1)


# ══════════════════════════════════════════════════════════
#  5.  Structural Inspection Planner (smoke tests)
# ══════════════════════════════════════════════════════════

class TestStructuralInspectionPlanner(unittest.TestCase):
    """Smoke tests for structural_inspection.py."""

    def setUp(self):
        # Import here to avoid early import errors if file is missing
        from structural_inspection import (
            Structure, InspectionConfig,
            StructuralInspectionPlanner, InspectionPhase,
        )
        self.Structure   = Structure
        self.Config      = InspectionConfig
        self.Planner     = StructuralInspectionPlanner
        self.Phase       = InspectionPhase

    def _make_planner(self, n=3):
        s = self.Structure(center_x=0, center_y=0, radius=10, height=30)
        c = self.Config(orbit_radius=25, alt_bands=3, orbit_points_per_sector=4)
        return self.Planner(s, c, n)

    def test_correct_number_of_assignments(self):
        p = self._make_planner(3)
        self.assertEqual(len(p.assignments), 3)

    def test_each_drone_has_scan_points(self):
        p = self._make_planner(4)
        for a in p.assignments:
            self.assertGreater(len(a.scan_points), 0)

    def test_start_inspection_changes_phase(self):
        p = self._make_planner(2)
        p.start_inspection()
        self.assertEqual(p.phase, self.Phase.SCANNING)

    def test_progress_starts_at_zero(self):
        p = self._make_planner(2)
        self.assertAlmostEqual(p.overall_progress, 0.0)

    def test_status_summary_has_expected_keys(self):
        p = self._make_planner(2)
        p.start_inspection()
        s = p.get_status_summary()
        for key in ("phase", "progress_pct", "elapsed_s",
                    "total_photos", "total_points", "drones"):
            self.assertIn(key, s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
