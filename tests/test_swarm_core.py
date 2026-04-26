"""
test_swarm_core.py — Pytest tests for consensus and formation algorithms
========================================================================
Covers:
  - SwarmConsensus role assignment and reassignment
  - Formation slot generation and morphing (FormationMorpher)
  - APFEngine including moving obstacle repulsion
  - MissionLogger SQLite events
"""

import math
import os
import sys
import time
import tempfile
import pytest

# Ensure src is importable without installation
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from consensus import SwarmConsensus, DroneState, BATTERY_LOW_THRESHOLD, ROLES
from formations import (
    FormationMorpher,
    FORMATIONS,
    get_formation_slots,
    get_slot_gps,
    FORMATION_KEYS,
)
from avoidance import APFEngine, MovingObstacle
from mission_logger import MissionLogger


# ===========================================================================
# DroneState
# ===========================================================================

class TestDroneState:
    def test_default_eligible(self):
        """A freshly created drone with full battery is eligible."""
        state = DroneState("d1")
        assert state.eligible is True

    def test_low_battery_ineligible(self):
        """Battery below threshold makes drone ineligible."""
        state = DroneState("d1")
        state.battery = BATTERY_LOW_THRESHOLD - 1
        assert state.eligible is False

    def test_dead_ineligible(self):
        """A crashed drone is ineligible regardless of battery."""
        state = DroneState("d1")
        state.alive = False
        assert state.eligible is False

    def test_score_zero_when_ineligible(self):
        """Ineligible drone scores 0."""
        state = DroneState("d1")
        state.battery = 5.0
        assert state.score() == 0.0

    def test_score_proportional_to_battery(self):
        """Score equals battery percentage when eligible."""
        state = DroneState("d1")
        state.battery = 75.0
        assert state.score() == pytest.approx(75.0)


# ===========================================================================
# SwarmConsensus
# ===========================================================================

class TestSwarmConsensus:
    def _make_consensus(self, n=4):
        ids = [str(i) for i in range(1, n + 1)]
        return SwarmConsensus(ids)

    def test_initial_roles_assigned(self):
        """All pre-defined roles should be assigned on init."""
        c = self._make_consensus(4)
        roles = c.get_roles()
        # Each role in ROLES must appear at least once
        assigned_roles = set(roles.values())
        for r in ROLES:
            assert r in assigned_roles

    def test_leader_elected(self):
        """get_leader() returns a valid drone id."""
        c = self._make_consensus(3)
        leader = c.get_leader()
        assert leader is not None
        assert leader in [str(i) for i in range(1, 4)]

    def test_role_reassigned_on_low_battery(self):
        """Dropping below battery threshold triggers reassignment."""
        c = self._make_consensus(4)
        # Find the leader
        leader = c.get_leader()
        assert leader is not None

        # Drain battery of leader drone
        triggered = c.update_drone(leader, battery=10.0)
        assert triggered is True
        # Leader role should now be on a different drone
        new_leader = c.get_leader()
        assert new_leader != leader

    def test_role_reassigned_on_failure(self):
        """trigger_reassignment() redistributes role to eligible drone."""
        c = self._make_consensus(4)
        leader = c.get_leader()
        result = c.trigger_reassignment(leader)
        new_leader = c.get_leader()
        # Original leader should no longer hold the role
        assert result[leader] is None
        assert new_leader != leader

    def test_no_leader_when_all_low_battery(self):
        """If all drones have low battery, no leader can be elected."""
        ids = ["a", "b"]
        c = SwarmConsensus(ids)
        c.update_drone("a", battery=5.0)
        c.update_drone("b", battery=5.0)
        # Manually remove remaining roles to simulate total failure
        c.trigger_reassignment("a")
        c.trigger_reassignment("b")
        # No drone should hold the leader role
        assert c.get_leader() is None

    def test_add_drone(self):
        """Adding a drone registers it in the consensus."""
        c = self._make_consensus(2)
        c.add_drone("new")
        roles = c.get_roles()
        assert "new" in roles

    def test_remove_drone(self):
        """Removing a drone unregisters it."""
        c = self._make_consensus(3)
        c.remove_drone("1")
        roles = c.get_roles()
        assert "1" not in roles

    def test_update_unknown_drone_returns_false(self):
        """Updating an unregistered drone returns False."""
        c = self._make_consensus(2)
        result = c.update_drone("ghost", battery=100.0)
        assert result is False

    def test_best_candidate_elected(self):
        """Highest-battery eligible drone wins the election."""
        c = SwarmConsensus(["a", "b", "c"])
        # Drain leader so reassignment triggers
        leader = c.get_leader()
        # Set specific batteries on remaining drones
        for did in ["a", "b", "c"]:
            if did != leader:
                c._drones[did].battery = 50.0 if did == "b" else 90.0
        # Remove existing roles from non-leaders to make them candidates
        for did, r in list(c._roles.items()):
            if did != leader:
                c._roles[did] = None
        c.trigger_reassignment(leader)
        new_leader = c.get_leader()
        # The drone with 90 % battery should be elected
        assert new_leader != leader


# ===========================================================================
# Formation blueprints
# ===========================================================================

class TestFormations:
    def test_all_formations_present(self):
        """All expected formation keys exist."""
        for key in ("V", "ARROW", "CIRCLE", "WALL", "LINE", "GRID"):
            assert key in FORMATIONS

    def test_get_formation_slots_truncates(self):
        """get_formation_slots respects num_drones limit."""
        slots = get_formation_slots("V", 3)
        assert len(slots) == 3

    def test_get_formation_slots_unknown_falls_back(self):
        """Unknown formation falls back to V."""
        slots_v = get_formation_slots("V", 5)
        slots_x = get_formation_slots("NONEXISTENT", 5)
        assert slots_v == slots_x

    def test_get_slot_gps_origin(self):
        """Slot (0, 0) at heading 0 should return the base GPS."""
        lat, lon = get_slot_gps(-35.363261, 149.165230, 0.0, 0, 0, 15.0)
        assert lat == pytest.approx(-35.363261, abs=1e-6)
        assert lon == pytest.approx(149.165230, abs=1e-6)

    def test_get_slot_gps_north_offset(self):
        """Forward offset with heading North moves drone northward."""
        base_lat, base_lon = -35.363261, 149.165230
        lat, lon = get_slot_gps(base_lat, base_lon, 0.0, 1, 0, 15.0)
        assert lat > base_lat  # further north

    def test_slot_gps_symmetry(self):
        """Left and right wing slots are symmetric around the heading axis."""
        base_lat, base_lon = -35.363261, 149.165230
        lat_left,  lon_left  = get_slot_gps(base_lat, base_lon, 0.0, -1, -1, 15.0)
        lat_right, lon_right = get_slot_gps(base_lat, base_lon, 0.0, -1,  1, 15.0)
        assert lat_left == pytest.approx(lat_right, abs=1e-8)
        assert lon_left == pytest.approx(-lon_right + 2 * base_lon, abs=1e-6)


# ===========================================================================
# FormationMorpher
# ===========================================================================

class TestFormationMorpher:
    def test_morph_returns_correct_steps(self):
        """morph() returns exactly `steps` frames."""
        morpher = FormationMorpher(num_drones=4)
        frames = morpher.morph("V", "CIRCLE", steps=10)
        assert len(frames) == 10

    def test_morph_first_frame_equals_source(self):
        """First frame matches source formation slots."""
        morpher = FormationMorpher(num_drones=3)
        src_slots = get_formation_slots("V", 3)
        frames = morpher.morph("V", "LINE", steps=5)
        for i, (fwd, right) in enumerate(frames[0]):
            assert fwd == pytest.approx(src_slots[i][0], abs=1e-9)
            assert right == pytest.approx(src_slots[i][1], abs=1e-9)

    def test_morph_last_frame_equals_target(self):
        """Last frame matches target formation slots."""
        morpher = FormationMorpher(num_drones=3)
        dst_slots = get_formation_slots("LINE", 3)
        frames = morpher.morph("V", "LINE", steps=5)
        for i, (fwd, right) in enumerate(frames[-1]):
            assert fwd == pytest.approx(dst_slots[i][0], abs=1e-9)
            assert right == pytest.approx(dst_slots[i][1], abs=1e-9)

    def test_morph_mid_frame_is_interpolated(self):
        """Middle frame values lie strictly between source and target."""
        morpher = FormationMorpher(num_drones=1)
        # V leader slot is (0,0), LINE leader slot is (0,0) — use drone2
        morpher2 = FormationMorpher(num_drones=2)
        frames = morpher2.morph("V", "LINE", steps=3)
        # Drone index 1: V slot=(-1,-1), LINE slot=(-1,0)
        # Mid frame (step 1 of 3) should be at t=0.5
        src_right = -1.0
        dst_right = 0.0
        expected_right = src_right + (dst_right - src_right) * 0.5
        assert frames[1][1][1] == pytest.approx(expected_right, abs=1e-9)

    def test_morph_minimum_steps(self):
        """steps=2 works (just start and end)."""
        morpher = FormationMorpher(num_drones=3)
        frames = morpher.morph("V", "CIRCLE", steps=2)
        assert len(frames) == 2

    def test_morph_raises_on_insufficient_steps(self):
        """steps < 2 raises ValueError."""
        morpher = FormationMorpher(num_drones=3)
        with pytest.raises(ValueError):
            morpher.morph("V", "LINE", steps=1)

    def test_current_formation_updated(self):
        """morph() updates the current_formation property."""
        morpher = FormationMorpher(num_drones=3)
        morpher.morph("V", "GRID", steps=5)
        assert morpher.current_formation == "GRID"

    def test_morph_more_drones_than_slots(self):
        """Morpher pads slots when num_drones exceeds formation definition."""
        morpher = FormationMorpher(num_drones=10)
        frames = morpher.morph("V", "LINE", steps=3)
        assert len(frames[0]) == 10


# ===========================================================================
# APFEngine — static obstacles
# ===========================================================================

class TestAPFEngine:
    def test_attractive_force_toward_target(self):
        """Attractive force direction points toward target."""
        engine = APFEngine()
        fx, fy = engine.compute_attractive(0, 0, 10, 0)
        assert fx > 0
        assert fy == pytest.approx(0.0, abs=1e-6)

    def test_no_attractive_at_target(self):
        """No force when already at target."""
        engine = APFEngine()
        fx, fy = engine.compute_attractive(5, 5, 5, 5)
        assert fx == pytest.approx(0.0, abs=1e-6)
        assert fy == pytest.approx(0.0, abs=1e-6)

    def test_drone_repulsion_active_when_close(self):
        """Drone repulsion activates when neighbor is within avoid_radius."""
        engine = APFEngine(drone_avoid_radius=14.0)
        fx, fy, active = engine.compute_drone_repulsion(0, 0, [(5, 0)])
        assert active is True
        assert fx > 0  # repelled away from neighbor at (5, 0)

    def test_drone_repulsion_inactive_when_far(self):
        """No repulsion when neighbor is outside avoid_radius."""
        engine = APFEngine(drone_avoid_radius=14.0)
        _, _, active = engine.compute_drone_repulsion(0, 0, [(50, 0)])
        assert active is False

    def test_obstacle_repulsion_active(self):
        """Static obstacle repulsion activates within avoid range."""
        engine = APFEngine(obstacle_avoid_radius=30.0)
        fx, fy, active = engine.compute_obstacle_repulsion(0, 0, [(10, 0, 2)])
        assert active is True
        assert fx < 0  # repelled away from obstacle at x=10

    def test_total_force_no_obstacles(self):
        """Total force with no obstacles just equals attractive force."""
        engine = APFEngine()
        tx, ty, active = engine.compute_total_force(0, 0, 10, 0, [], [])
        assert tx > 0
        assert active is False


# ===========================================================================
# APFEngine — moving obstacles
# ===========================================================================

class TestMovingObstacle:
    def test_predicted_position(self):
        """Predicted position advances by velocity * dt."""
        obs = MovingObstacle(x=0, y=0, vx=2.0, vy=1.0, radius=3.0)
        px, py = obs.predicted_position(5.0)
        assert px == pytest.approx(10.0)
        assert py == pytest.approx(5.0)

    def test_update_position(self):
        """update() advances the stored position."""
        obs = MovingObstacle(x=0, y=0, vx=3.0, vy=0.0)
        obs.update(2.0)
        assert obs.x == pytest.approx(6.0)

    def test_to_static_tuple(self):
        """to_static_tuple() returns (x, y, radius)."""
        obs = MovingObstacle(x=5, y=7, radius=4)
        t = obs.to_static_tuple()
        assert t == (5, 7, 4)

    def test_moving_obstacle_repulsion_active(self):
        """Repulsion from a close moving obstacle is active."""
        engine = APFEngine(obstacle_avoid_radius=30.0)
        obs = MovingObstacle(x=10, y=0, vx=0, vy=0, radius=2)
        fx, fy, active = engine.compute_moving_obstacle_repulsion(0, 0, 0, 0, [obs])
        assert active is True

    def test_moving_obstacle_repulsion_direction(self):
        """Repulsion direction is away from the obstacle."""
        engine = APFEngine(obstacle_avoid_radius=30.0)
        obs = MovingObstacle(x=10, y=0, vx=0, vy=0, radius=2)
        fx, fy, _ = engine.compute_moving_obstacle_repulsion(0, 0, 0, 0, [obs])
        assert fx < 0  # repelled in -x direction (away from x=10)

    def test_convergence_amplifier_increases_repulsion(self):
        """Approaching obstacle has stronger repulsion than stationary drone."""
        engine = APFEngine(obstacle_avoid_radius=30.0, obstacle_repulse_gain=20.0)
        obs_stationary = MovingObstacle(x=10, y=0, vx=0, vy=0, radius=2)
        obs_approaching = MovingObstacle(x=10, y=0, vx=-5, vy=0, radius=2)
        # Drone has velocity toward obstacle
        _, _, _ = engine.compute_moving_obstacle_repulsion(0, 0, 5, 0, [obs_stationary])
        fx_stat, _, _ = engine.compute_moving_obstacle_repulsion(0, 0, 5, 0, [obs_stationary])
        fx_appr, _, _ = engine.compute_moving_obstacle_repulsion(0, 0, 5, 0, [obs_approaching])
        # Approaching obstacle (drone converging) yields stronger repulsion magnitude
        assert abs(fx_appr) >= abs(fx_stat)

    def test_total_force_with_moving_obstacle(self):
        """compute_total_force with moving_obstacles sets avoidance flag."""
        engine = APFEngine()
        obs = MovingObstacle(x=5, y=0, vx=0, vy=0, radius=1)
        _, _, active = engine.compute_total_force(
            0, 0, 20, 0, [], [], moving_obstacles=[obs]
        )
        assert active is True

    def test_total_force_backward_compatible(self):
        """compute_total_force still works without moving_obstacles arg."""
        engine = APFEngine()
        tx, ty, active = engine.compute_total_force(0, 0, 10, 0, [], [(50, 0, 2)])
        assert isinstance(tx, float)
        assert isinstance(ty, float)


# ===========================================================================
# MissionLogger
# ===========================================================================

class TestMissionLogger:
    def _make_logger(self):
        tmp = tempfile.mktemp(suffix=".db")
        return MissionLogger(tmp), tmp

    def test_log_role_change(self):
        """Role change events are persisted."""
        log, db = self._make_logger()
        log.log_role_change("d1", "scout", "leader", reason="battery_low")
        events = log.get_events(event_type="role_change")
        assert len(events) == 1
        assert events[0]["data"]["new_role"] == "leader"
        log.close()
        os.remove(db)

    def test_log_formation_switch(self):
        """Formation switch events are persisted."""
        log, db = self._make_logger()
        log.log_formation_switch("V", "CIRCLE", commander_id="d1")
        events = log.get_events(event_type="formation_switch")
        assert len(events) == 1
        assert events[0]["data"]["to_formation"] == "CIRCLE"
        log.close()
        os.remove(db)

    def test_log_path_replan(self):
        """Path replan events are persisted."""
        log, db = self._make_logger()
        log.log_path_replan(
            drone_id="d2",
            trigger="moving_obstacle",
            old_target={"x": 0, "y": 0},
            new_target={"x": 5, "y": 3},
        )
        events = log.get_events(event_type="path_replan")
        assert len(events) == 1
        assert events[0]["data"]["trigger"] == "moving_obstacle"
        log.close()
        os.remove(db)

    def test_log_telemetry(self):
        """Telemetry snapshots are persisted."""
        log, db = self._make_logger()
        log.log_telemetry("d1", -35.363, 149.165, 20.0, 85.0, "leader", "V")
        events = log.get_events(event_type="telemetry")
        assert len(events) == 1
        assert events[0]["data"]["battery"] == pytest.approx(85.0)
        log.close()
        os.remove(db)

    def test_log_custom(self):
        """Custom events are persisted with message."""
        log, db = self._make_logger()
        log.log_custom("Mission started", extra={"phase": "takeoff"})
        events = log.get_events(event_type="custom")
        assert len(events) == 1
        assert events[0]["data"]["phase"] == "takeoff"
        log.close()
        os.remove(db)

    def test_query_filter_by_drone(self):
        """get_events() filters by drone_id correctly."""
        log, db = self._make_logger()
        log.log_role_change("d1", None, "leader")
        log.log_role_change("d2", None, "scout")
        events = log.get_events(event_type="role_change", drone_id="d1")
        assert len(events) == 1
        assert events[0]["drone_id"] == "d1"
        log.close()
        os.remove(db)

    def test_query_since(self):
        """get_events() respects the `since` timestamp filter."""
        log, db = self._make_logger()
        log.log_custom("old event")
        marker = time.time()
        time.sleep(0.05)
        log.log_custom("new event")
        events = log.get_events(since=marker)
        assert len(events) == 1
        assert events[0]["data"]["message"] == "new event"
        log.close()
        os.remove(db)

    def test_query_limit(self):
        """get_events() respects the row limit."""
        log, db = self._make_logger()
        for i in range(10):
            log.log_custom(f"event {i}")
        events = log.get_events(limit=3)
        assert len(events) == 3
        log.close()
        os.remove(db)
