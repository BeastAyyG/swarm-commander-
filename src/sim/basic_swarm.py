import math
import time

class Drone:
    def __init__(self, drone_id, x=0.0, y=0.0):
        self.drone_id = drone_id
        self.x = x
        self.y = y
        self.target_x = x
        self.target_y = y
        self.speed = 2.0  # m/s movement speed

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y

    def update(self, dt):
        """Simulates the drone moving towards its target."""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        
        if dist > 0.05:
            move_x = (dx / dist) * self.speed * dt
            move_y = (dy / dist) * self.speed * dt
            
            # Prevent overshooting the target
            if abs(move_x) > abs(dx): move_x = dx
            if abs(move_y) > abs(dy): move_y = dy
                
            self.x += move_x
            self.y += move_y
            
    def __str__(self):
        return f"Drone {self.drone_id} | Pos: ({self.x:.2f}, {self.y:.2f}) -> Target: ({self.target_x:.2f}, {self.target_y:.2f})"

class SwarmCoordinator:
    def __init__(self, leader):
        self.leader = leader
        self.followers = []
        
        # Pre-defined formation offsets relative to the leader (x, y)
        self.formations = {
            'V': [(-2, -2), (2, -2), (-4, -4), (4, -4)],
            'LINE': [(0, -2), (0, -4), (0, -6), (0, -8)],
            'ECHELON_RIGHT': [(2, -2), (4, -4), (6, -6), (8, -8)]
        }
        self.current_formation = 'V'

    def add_follower(self, drone):
        self.followers.append(drone)

    def set_formation(self, formation_name):
        if formation_name in self.formations:
            self.current_formation = formation_name
            print(f"--> Swarm formation changed to {formation_name}")
        else:
            print(f"Unknown formation: {formation_name}")

    def calculate_targets(self):
        """Calculates where each follower should be based on the leader's position."""
        offsets = self.formations[self.current_formation]
        
        for idx, follower in enumerate(self.followers):
            if idx < len(offsets):
                off_x, off_y = offsets[idx]
                
                # In a real flight controller, this would involve rotating
                # the offsets based on the leader's current heading (yaw).
                follower.target_x = self.leader.x + off_x
                follower.target_y = self.leader.y + off_y

    def update_swarm(self, dt):
        """Updates physics/positions for all drones in the swarm."""
        self.leader.update(dt)
        self.calculate_targets()
        for follower in self.followers:
            follower.update(dt)

def run_simulation():
    # 1. Initialize Leader
    leader = Drone(0, x=0, y=0)
    swarm = SwarmCoordinator(leader)
    
    # 2. Add Followers starting at random scattered positions
    swarm.add_follower(Drone(1, x=-5, y=-5))
    swarm.add_follower(Drone(2, x=5, y=-10))
    swarm.add_follower(Drone(3, x=-8, y=2))
        
    print("=== Initial Swarm State ===")
    print(leader)
    for f in swarm.followers: print(f)
        
    print("\n=== Initiating Swarm Movement (Target: 10, 20) ===")
    leader.set_target(10, 20)
    
    # Simulate for 15 steps
    for step in range(15):
        swarm.update_swarm(dt=1.0)
        
        # Change formation midway just to demonstrate!
        if step == 7:
            print("\n!!! Switching Formation !!!")
            swarm.set_formation('LINE')

        print(f"\n[Step {step + 1}]")
        print(leader)
        for f in swarm.followers:
            print(f)
            
if __name__ == "__main__":
    run_simulation()
