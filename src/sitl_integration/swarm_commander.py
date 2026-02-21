#!/usr/bin/env python3
"""
SWARM COMMANDER
Broadcasts global swarm state and formation assignments via UDP to Drone Agents.

Simulates a Ground Control Station routing targets over a mesh network.
"""

import socket
import json
import time
import math
import argparse
try:
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("WARNING: Scipy not installed. Falling back to greedy task allocation.")

# --- Configuration ---
BROADCAST_IP = '127.0.0.1'  # Localhost for SITL testing
BROADCAST_PORT = 5005
SYNC_RATE = 5               # Hz (Transmit 5 times a second)

class SwarmCommander:
    def __init__(self, num_drones=3):
        self.num_drones = num_drones
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Swarm Global State
        # Starting roughly at SITL default (Canberra)
        self.global_target_lat = -35.363261
        self.global_target_lon = 149.165230
        self.global_target_alt = 20.0 # Meters AGL
        self.global_heading = 0.0 # Radians (North = 0)
        
        self.active_formation = 'V'
        
        # Blueprint: offsets (forward_meters, right_meters)
        self.spacing = 15.0 # meters between drones
        self.blueprints = {
            'V': [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2), (-3,-3), (-3,3)],
            'LINE': [(0,0), (-1,0), (-2,0), (-3,0), (-4,0)],
            'SQUARE': [(1,1), (1,-1), (-1,1), (-1,-1), (0,0)]
        }
        
        # State tracking for Auction algorithm
        # Dict mapping drone_id -> current (lat, lon) -> received via telemetry
        self.drone_telemetry = {}
        # Dict mapping drone_id -> assigned slot (fwd_offset, right_offset)
        self.assignments = {i: self.blueprints['V'][i % len(self.blueprints['V'])] for i in range(1, num_drones+1)}

    def run_auction(self):
        """
        Dynamically reassigns slots to minimize overall distance if we had telemetry.
        For this SITL demo without a complex 2-way ROS mesh, we statically map IDs to slots
        based on the blueprint length.
        
        To fully implement Hungarian here, the Commander needs a UDP *receiver* thread
        to get `drone_telemetry` from the agents.
        """
        blueprint = self.blueprints[self.active_formation]
        slots_needed = blueprint[:min(self.num_drones, len(blueprint))]
        
        # Simple static mapping for demo:
        for i in range(1, self.num_drones + 1):
            if i <= len(slots_needed):
                self.assignments[i] = slots_needed[i-1]
            else:
                self.assignments[i] = (0, 0) # Fallback

    def set_formation(self, formation_name):
        if formation_name in self.blueprints:
            self.active_formation = formation_name
            self.run_auction()
            print(f"[COMMANDER] Transitioning swarm to: {formation_name}")

    def update_target(self, lat, lon, alt):
        self.global_target_lat = lat
        self.global_target_lon = lon
        self.global_target_alt = alt

    def broadcast_state(self):
        # Package the assignments
        payload = {
            "timestamp": time.time(),
            "target_lat": self.global_target_lat,
            "target_lon": self.global_target_lon,
            "target_alt": self.global_target_alt,
            "heading": self.global_heading,
            "formation": self.active_formation,
            "assignments": self.assignments # ID: (fwd, right)
        }
        
        message = json.dumps(payload).encode('utf-8')
        self.sock.sendto(message, (BROADCAST_IP, BROADCAST_PORT))
        # print(f"[TX] Broadcasted heartbeat. Target: {self.global_target_lat:.6f}, {self.global_target_lon:.6f}")

def main():
    parser = argparse.ArgumentParser("Swarm Commander")
    parser.add_argument("--drones", type=int, default=3, help="Number of drones in swarm")
    args = parser.parse_args()
    
    commander = SwarmCommander(num_drones=args.drones)
    
    print("=========================================")
    print(" SWARM COMMANDER UPLINK ACTIVE           ")
    print(f" UDP Broadcast: {BROADCAST_IP}:{BROADCAST_PORT} ")
    print("=========================================")
    
    try:
        # Simulate a mission
        print("Waiting 10 seconds for agents to initialize and takeoff...")
        time.sleep(10)
        
        print("\n[MISSION] Swarm Target: Move North (Lat += 0.001)")
        commander.set_formation('V')
        commander.update_target(-35.362261, 149.165230, 20.0)
        
        for _ in range(10 * SYNC_RATE): # 10 seconds
            commander.broadcast_state()
            time.sleep(1.0 / SYNC_RATE)
            
        print("\n[MISSION] Swarm Target: Morph to LINE and move East")
        commander.set_formation('LINE')
        commander.global_heading = math.pi / 2 # Pointing East
        commander.update_target(-35.362261, 149.166230, 20.0)
        
        for _ in range(15 * SYNC_RATE): # 15 seconds
            commander.broadcast_state()
            time.sleep(1.0 / SYNC_RATE)
            
        print("\n[MISSION] Return to Launch (RTL)")
        commander.set_formation('SQUARE')
        commander.update_target(-35.363261, 149.165230, 20.0)
        
        while True:
            commander.broadcast_state()
            time.sleep(1.0 / SYNC_RATE)

    except KeyboardInterrupt:
        print("\n[COMMANDER] Shutting down uplink.")

if __name__ == "__main__":
    main()
