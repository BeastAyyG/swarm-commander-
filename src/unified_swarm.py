#!/usr/bin/env python3
"""
UNIFIED SWARM LAUNCHER
Spawns SITL instances, connects agents, and runs the mission — ALL in ONE process.
No multiple terminals needed.
"""

import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

import subprocess
import os
import sys
import time
import math
import json
import socket
import signal
import threading
import shutil

from dronekit import connect, VehicleMode, LocationGlobalRelative

# ==================== CONFIGURATION ====================
NUM_DRONES = 3
ARDUCOPTER_BIN = "/home/q/ardupilot/build/sitl/bin/arducopter"
HOME_LOCATION = "-35.363261,149.165230,584,353"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BROADCAST_PORT = 5005
FORMATION_SPACING = 15.0  # meters

# ==================== FORMATION BLUEPRINTS ====================
BLUEPRINTS = {
    'V':      [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2)],
    'LINE':   [(0,0), (-1,0), (-2,0), (-3,0), (-4,0)],
    'SQUARE': [(1,1), (1,-1), (-1,1), (-1,-1), (0,0)],
}

# ==================== HELPER: GPS OFFSET ====================
def offset_location(base_lat, base_lon, base_alt, d_north, d_east):
    """Returns (lat, lon, alt) offset by d_north/d_east meters."""
    earth_radius = 6378137.0
    new_lat = base_lat + (d_north / earth_radius) * (180.0 / math.pi)
    new_lon = base_lon + (d_east / (earth_radius * math.cos(math.pi * base_lat / 180.0))) * (180.0 / math.pi)
    return new_lat, new_lon, base_alt

# ==================== SITL PROCESS MANAGER ====================
sitl_processes = []

def launch_sitl_instances(num_drones):
    """Launch raw arducopter binaries."""
    print(f"\n{'='*50}")
    print(f"  PHASE 1: Launching {num_drones} SITL Instances")
    print(f"{'='*50}")
    
    for i in range(num_drones):
        instance_dir = os.path.join(BASE_DIR, f"sitl_unified_{i+1}")
        if os.path.exists(instance_dir):
            shutil.rmtree(instance_dir)
        os.makedirs(instance_dir)
        
        # Write minimal params
        parm_file = os.path.join(instance_dir, "default.parm")
        with open(parm_file, 'w') as f:
            f.write(f"SYSID_THISMAV {i+1}\n")
            f.write("FRAME_CLASS 1\n")
            f.write("FRAME_TYPE 1\n")
            f.write("ARMING_CHECK 0\n")
            f.write("SIM_SPEEDUP 1\n")
            f.write("BRD_SAFETYENABLE 0\n")
            f.write("BATT_MONITOR 0\n")
            f.write("DISARM_DELAY 0\n")
        
        port = 5760 + i * 10
        print(f"  [SITL] Drone {i+1} → tcp:127.0.0.1:{port}")
        
        proc = subprocess.Popen(
            [
                ARDUCOPTER_BIN,
                f"-I{i}",
                "--model", "quad",
                "--home", HOME_LOCATION,
                "--defaults", parm_file,
            ],
            cwd=instance_dir,
            stdout=open(os.path.join(instance_dir, "sitl.log"), 'w'),
            stderr=subprocess.STDOUT,
        )
        sitl_processes.append(proc)
    
    print(f"\n  Waiting 15 seconds for SITL EKF initialization...")
    time.sleep(15)
    
    # Verify processes are alive
    alive = sum(1 for p in sitl_processes if p.poll() is None)
    print(f"  ✓ {alive}/{num_drones} SITL instances running\n")
    return alive == num_drones

# ==================== DRONE AGENT ====================
def connect_drone(drone_id, port):
    """Connect to a single SITL instance with retries."""
    conn_str = f"tcp:127.0.0.1:{port}"
    print(f"  [AGENT {drone_id}] Connecting to {conn_str}...")
    
    vehicle = None
    for attempt in range(10):
        try:
            vehicle = connect(conn_str, wait_ready=True, timeout=30)
            print(f"  [AGENT {drone_id}] ✓ Connected! Firmware: {vehicle.version}")
            return vehicle
        except Exception as e:
            print(f"  [AGENT {drone_id}] Attempt {attempt+1}/10 failed: {type(e).__name__}")
            time.sleep(3)
    
    print(f"  [AGENT {drone_id}] ✗ FAILED to connect after 10 attempts")
    return None

def calibrate_accel(vehicle, drone_id):
    """Send MAVLink accel calibration command to bypass '3D Accel calibration needed'."""
    print(f"  [AGENT {drone_id}] Running accelerometer calibration...")
    from pymavlink import mavutil
    
    # MAV_CMD_PREFLIGHT_CALIBRATION (241)
    # param5=4 means "simple accelerometer calibration" 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,       # confirmation
        0,       # param1: gyro cal (0=no)
        0,       # param2: magnetometer (0=no)
        0,       # param3: ground pressure (0=no) 
        0,       # param4: radio (0=no)
        4,       # param5: accel cal (4=simple cal)
        0,       # param6: compass/motor (0=no)
        0,       # param7: (unused)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    # Wait for calibration to complete and gyros/accels to settle
    print(f"  [AGENT {drone_id}] Waiting 15s for gyros/accels to settle...")
    time.sleep(15)
    print(f"  [AGENT {drone_id}] ✓ Calibration complete")

def arm_and_takeoff(vehicle, drone_id, target_alt=20.0):
    """Arm and takeoff a single drone."""
    # First, calibrate accelerometers
    calibrate_accel(vehicle, drone_id)
    
    print(f"  [AGENT {drone_id}] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    
    # Wait for EKF to be happy (GPS lock + position estimate)
    print(f"  [AGENT {drone_id}] Waiting for EKF/GPS lock...")
    timeout = 120
    start = time.time()
    while time.time() - start < timeout:
        # Check if we can get a valid location
        loc = vehicle.location.global_relative_frame
        if loc and loc.lat != 0.0:
            # Also check EKF status if available
            try:
                ekf_ok = vehicle.ekf_ok
                if ekf_ok:
                    print(f"  [AGENT {drone_id}] ✓ EKF OK, GPS locked")
                    break
            except:
                pass
        time.sleep(2)
    else:
        print(f"  [AGENT {drone_id}] ⚠ EKF timeout, trying to arm anyway...")
    
    print(f"  [AGENT {drone_id}] Arming...")
    vehicle.armed = True
    
    timeout = 30
    start = time.time()
    while not vehicle.armed and time.time() - start < timeout:
        vehicle.armed = True  # Keep trying
        time.sleep(1)
    
    if not vehicle.armed:
        print(f"  [AGENT {drone_id}] ✗ Failed to arm (PreArm check failed)")
        return False
    
    print(f"  [AGENT {drone_id}] ✓ Armed! Taking off to {target_alt}m...")
    vehicle.simple_takeoff(target_alt)
    
    timeout = 60
    start = time.time()
    while time.time() - start < timeout:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None and alt >= target_alt * 0.95:
            print(f"  [AGENT {drone_id}] ✓ Reached {alt:.1f}m")
            break
        time.sleep(1)
    
    return True

def goto_formation_slot(vehicle, drone_id, target_lat, target_lon, heading, slot_fwd, slot_right, spacing):
    """Navigate drone to its formation slot."""
    # Rotate slot offset by heading
    rotated_north = (slot_fwd * math.cos(heading) - slot_right * math.sin(heading)) * spacing
    rotated_east  = (slot_fwd * math.sin(heading) + slot_right * math.cos(heading)) * spacing
    
    my_lat, my_lon, my_alt = offset_location(target_lat, target_lon, 20.0, rotated_north, rotated_east)
    target = LocationGlobalRelative(my_lat, my_lon, 20.0)
    vehicle.simple_goto(target)

# ==================== MISSION RUNNER ====================
def run_mission(vehicles):
    """Execute a predefined swarm mission."""
    n = len(vehicles)
    
    print(f"\n{'='*50}")
    print(f"  PHASE 4: Running Swarm Mission ({n} drones)")
    print(f"{'='*50}")
    
    # Mission waypoints: (description, formation, target_lat, target_lon, heading_rad, duration_sec)
    mission = [
        ("Moving North in V-Formation",     'V',      -35.362261, 149.165230, 0,           15),
        ("Morphing to LINE, heading East",   'LINE',   -35.362261, 149.166230, math.pi/2,   15),
        ("Morphing to SQUARE, heading South",'SQUARE', -35.363261, 149.166230, math.pi,     15),
        ("Returning Home in V",              'V',      -35.363261, 149.165230, 0,           15),
    ]
    
    for desc, formation, t_lat, t_lon, heading, duration in mission:
        print(f"\n  ► {desc}")
        blueprint = BLUEPRINTS[formation]
        
        for i, vehicle in enumerate(vehicles):
            if i < len(blueprint):
                slot_fwd, slot_right = blueprint[i]
                goto_formation_slot(vehicle, i+1, t_lat, t_lon, heading, slot_fwd, slot_right, FORMATION_SPACING)
        
        # Hold for duration, printing telemetry every 3 seconds
        for t in range(0, duration, 3):
            time.sleep(3)
            telemetry = []
            for i, v in enumerate(vehicles):
                loc = v.location.global_relative_frame
                telemetry.append(f"D{i+1}: ({loc.lat:.6f}, {loc.lon:.6f}, {loc.alt:.1f}m)")
            print(f"    [{t+3}s] {' | '.join(telemetry)}")
    
    print(f"\n  ✓ Mission Complete! Commanding RTL...")
    for i, v in enumerate(vehicles):
        v.mode = VehicleMode("RTL")
        print(f"  [AGENT {i+1}] → RTL")

# ==================== CLEANUP ====================
def cleanup(sig=None, frame=None):
    """Kill all SITL processes."""
    print("\n\nCleaning up...")
    for proc in sitl_processes:
        try:
            proc.kill()
        except:
            pass
    subprocess.run(["pkill", "-9", "-f", "arducopter"], capture_output=True)
    print("Done.")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ==================== MAIN ====================
def main():
    print("""
╔══════════════════════════════════════════════════╗
║     🚀 UNIFIED SWARM SITL LAUNCHER 🚀          ║
║     ArduPilot + DroneKit + Formation Control     ║
╚══════════════════════════════════════════════════╝
    """)
    
    # Phase 1: Launch SITL
    if not launch_sitl_instances(NUM_DRONES):
        print("ERROR: Not all SITL instances started. Check logs.")
        cleanup()
        return
    
    # Phase 2: Connect Agents
    print(f"{'='*50}")
    print(f"  PHASE 2: Connecting Drone Agents")
    print(f"{'='*50}")
    
    vehicles = []
    for i in range(NUM_DRONES):
        port = 5760 + i * 10
        v = connect_drone(i+1, port)
        if v:
            vehicles.append(v)
        else:
            print(f"\n  WARNING: Drone {i+1} failed to connect. Continuing with {len(vehicles)} drones.")
    
    if not vehicles:
        print("ERROR: No drones connected. Aborting.")
        cleanup()
        return
    
    # Phase 3: Arm and Takeoff
    print(f"\n{'='*50}")
    print(f"  PHASE 3: Arming & Takeoff ({len(vehicles)} drones)")
    print(f"{'='*50}")
    
    armed_vehicles = []
    for i, v in enumerate(vehicles):
        if arm_and_takeoff(v, i+1):
            armed_vehicles.append(v)
    
    if not armed_vehicles:
        print("ERROR: No drones could arm. Check 'sitl_unified_*/sitl.log' for details.")
        cleanup()
        return
    
    # Phase 4: Mission
    run_mission(armed_vehicles)
    
    # Wait a bit for RTL
    print("\n  Waiting 20 seconds for RTL...")
    time.sleep(20)
    
    # Cleanup
    for v in vehicles:
        try:
            v.close()
        except:
            pass
    
    cleanup()

if __name__ == "__main__":
    main()
