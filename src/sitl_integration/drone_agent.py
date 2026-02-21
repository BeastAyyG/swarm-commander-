#!/usr/bin/env python3
"""
DRONE AGENT (Edge Node / Companion Computer)
Runs on each individual drone. Receives UDP targets from Swarm Commander
and commands the local Pixhawk (SITL) via DroneKit/MAVLink.
"""

import socket
import json
import time
import math
import argparse
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

from dronekit import connect, VehicleMode, LocationGlobalRelative

# Broadcast settings matching Commander
LISTEN_IP = '0.0.0.0'
LISTEN_PORT = 5005

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative exactly dNorth/dEast meters from original_location
    Uses approximations valid over small distances.
    """
    earth_radius = 6378137.0 # Radius of "spherical" earth
    
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    
    newlat = original_location.lat + (dLat * 180.0 / math.pi)
    newlon = original_location.lon + (dLon * 180.0 / math.pi)
    
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

class DroneAgent:
    def __init__(self, node_id, connection_string):
        self.node_id = str(node_id) # Commander indexes by string '1', '2' etc through JSON
        self.connection_string = connection_string
        
        print(f"[AGENT {self.node_id}] Connecting to vehicle on: {connection_string}")
        self.vehicle = None
        for i in range(12): # Try for 60 seconds
            try:
                self.vehicle = connect(connection_string, wait_ready=True, timeout=30)
                break
            except Exception as e:
                print(f"  [AGENT {self.node_id}] Connection attempt {i+1} failed ({e}). Retrying in 5s...")
                time.sleep(5)
        
        if not self.vehicle:
            print(f"CRITICAL: Agent {self.node_id} could not connect to SITL after multiple attempts.")
            exit(1)
            
        print(f"[AGENT {self.node_id}] Connected! Firmware: {self.vehicle.version}")
            
        # UDP Receiver setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Allow multiple agents on same machine to listen to same broadcast
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.setblocking(False)
        
        # Internal State
        self.target_global = None # Target from commander
        self.active = False
        self.last_msg_time = 0

    def arm_and_takeoff(self, target_altitude):
        """Standard ArduPilot takeoff sequence"""
        print(f"[AGENT {self.node_id}] Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(f" Agent {self.node_id} waiting for vehicle to initialize...")
            time.sleep(1)

        print(f"[AGENT {self.node_id}] Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(f" Agent {self.node_id} waiting for arming...")
            time.sleep(1)

        print(f"[AGENT {self.node_id}] Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f" Agent {self.node_id} Altitude: {alt}")
            if alt >= target_altitude * 0.95:
                print(f"[AGENT {self.node_id}] Reached target altitude")
                break
            time.sleep(1)

    def listen_for_commander(self):
        try:
            data, addr = self.sock.recvfrom(4096)
            msg = json.loads(data.decode('utf-8'))
            self.last_msg_time = time.time()
            return msg
        except BlockingIOError:
            return None # No data available right now

    def process_swarm_logic(self, payload):
        """
        Calculates local target based on Global Target + assigned offset + heading.
        """
        # 1. Parse Global Goal
        g_lat = payload['target_lat']
        g_lon = payload['target_lon']
        g_alt = payload['target_alt']
        g_heading = payload['heading'] # Radians
        
        assignments = payload['assignments']
        
        if self.node_id not in assignments:
            # We are not assigned a slot in this formation. 
            # In a real scenario, RTB logic would trigger here.
            return
            
        # 2. Get my assigned offset (Forward/Right in meters)
        fwd, right = assignments[self.node_id]
        
        # 3. Rotate offset to align with Swarm Heading (2D Rotation Matrix)
        rotated_fwd = fwd * math.cos(g_heading) - right * math.sin(g_heading)
        rotated_east = fwd * math.sin(g_heading) + right * math.cos(g_heading)
        
        # 4. Calculate actual GPS target coordinate
        # The base coordinate is the commander's global target
        base_loc = LocationGlobalRelative(g_lat, g_lon, g_alt)
        
        # Offset to find our specific formation slot
        my_target = get_location_metres(base_loc, rotated_fwd, rotated_east)
        self.target_global = my_target
        
        # print(f"[AGENT {self.node_id}] New Target calculated (Offsets: {fwd}, {right})")

    def execution_loop(self):
        """Main control loop running at roughly 10Hz"""
        try:
            while True:
                # 1. Network: Check for Commander updates
                msg = self.listen_for_commander()
                if msg:
                    self.process_swarm_logic(msg)
                    self.active = True
                    
                # 2. Safety: Failsafe if lost connection to GCS for 5 seconds
                if self.active and (time.time() - self.last_msg_time > 5.0):
                    print(f"[WARNING] Agent {self.node_id} lost GCS link! Holding position.")
                    # In real life, might set RTL here
                    self.active = False 
                    
                # 3. Flight Control: Send MAVLink commands
                if self.active and self.target_global and self.vehicle.mode.name == "GUIDED":
                    # For simplicity, using simple_goto to command absolute position.
                    # Advanced version uses `send_nav_velocity` paired with APF logic.
                    self.vehicle.simple_goto(self.target_global)
                    
                time.sleep(0.1) # Run at 10Hz
                
        except KeyboardInterrupt:
            print(f"\n[AGENT {self.node_id}] Landing...")
            self.vehicle.mode = VehicleMode("RTL")

def main():
    parser = argparse.ArgumentParser("DroneAgent - SITL Link")
    parser.add_argument("--id", type=int, required=True, help="Unique Drone ID for swarm bidding (1, 2, 3...)")
    parser.add_argument("--connect", type=str, required=True, help="DroneKit connection string (e.g., tcp:127.0.0.1:5760)")
    args = parser.parse_args()
    
    agent = DroneAgent(args.id, args.connect)
    
    # Wait to arm until Commander is broadcasting
    print(f"\n[AGENT {args.id}] Waiting for initial Commander Heartbeat before takeoff...")
    while True:
        if agent.listen_for_commander():
            break
        time.sleep(0.5)
        
    print(f"[AGENT {args.id}] Uplink verified. Initiating Takeoff Sequence.")
    agent.arm_and_takeoff(20.0) # Typical SITL starting altitude
    
    print(f"[AGENT {args.id}] Swarm execution loop active. Waiting for waypoints.")
    agent.execution_loop()
    
if __name__ == "__main__":
    main()
