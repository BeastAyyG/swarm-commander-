"""
REALISTIC SWARM ARCHITECTURE & SIMULATION
=========================================

How this translates to real-world ArduPilot / ROS2 deployments:

1. The Ground Control Station (GCS) / Swarm Commander:
   - In reality, this is a computer running MAVProxy or ROS2.
   - It DOES NOT control each drone's motors. Instead, it acts as a broadcaster.
   - It transmits a synchronous "Swarm State Heartbeat" at a fixed rate (e.g., 10 Hz) over WiFi/Telemetry.
   - The broadcast contains: [Global Swarm Target X/Y, Swarm Heading, Active Formation].

2. The Drone (Edge Computer + Flight Controller):
   - Each drone has a unique ID (0, 1, 2, 3...) and knows its role.
   - The Companion Computer (e.g., Raspberry Pi) receives the Swarm State broadcast.
   - Decentralized Calculation: The drone calculates its OWN target coordinate locally by applying its specific formation offset to the Global Swarm Target.
   - The Edge Computer then sends MAVLink `SET_POSITION_TARGET_LOCAL_NED` commands to its Flight Controller (Pixhawk).
   - The Flight Controller executes the physical movement using its internal PID tuning, obeying realistic acceleration, velocity, and jerk limits.

This simulation models the above architecture:
- Drones have realistic physics (inertia, acceleration limits, turn rates).
- The "Mouse" acts as the GCS broadcasting the global waypoint.
- Drones independently compute their targets and navigate there realistically.
"""

import pygame
import math
import sys
import random

# --- Constants & Config ---
WIDTH, HEIGHT = 900, 700
FPS = 60
SYNC_RATE = 10 # Hz, how often the GCS updates the drones (simulating network)

# Colors
BG_COLOR = (20, 24, 32)
DRONE_COLOR = (70, 200, 150)
LEADER_COLOR = (255, 100, 100)
TARGET_COLOR = (255, 255, 255, 100) # Alpha for transparency
GCS_COLOR = (100, 150, 255)

# --- Kinematic Limits (Simulating real quadcopters) ---
MAX_SPEED = 200.0        # px/sec
MAX_ACCEL = 300.0        # px/sec^2
MAX_TURN_RATE = 4.0      # rad/sec (Realistic yaw rate)
DRONE_SIZE = 10


def constrain_angle(angle):
    """Keep angle between -PI and PI"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class RealisticDrone:
    def __init__(self, node_id, x, y):
        self.node_id = node_id
        
        # Physical State
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0 # Radians
        
        # Controller State (received from Edge Computer)
        self.target_x = float(x)
        self.target_y = float(y)
        
        # Trail for visualization
        self.trail = []

    def receive_target(self, tx, ty):
        """Simulates receiving a MAVLink position target command."""
        self.target_x = tx
        self.target_y = ty

    def update_physics(self, dt):
        """Internal flight controller simulation (PID & Kinematics)"""
        
        # 1. Navigation Vector to Target
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        
        # 2. Desired Velocity (Proportional to distance, capped at MAX_SPEED)
        p_gain = 2.0 # Proportional tuning
        desired_speed = min(MAX_SPEED, dist * p_gain)
        
        if dist > 1.0:
            desired_vx = (dx / dist) * desired_speed
            desired_vy = (dy / dist) * desired_speed
        else:
            desired_vx, desired_vy = 0, 0
            
        # 3. Acceleration Control (Limiting how fast velocity changes - Inertia)
        ax = (desired_vx - self.vx) * 3.0 # acceleration required
        ay = (desired_vy - self.vy) * 3.0
        
        accel_mag = math.hypot(ax, ay)
        if accel_mag > MAX_ACCEL:
            ax = (ax / accel_mag) * MAX_ACCEL
            ay = (ay / accel_mag) * MAX_ACCEL
            
        # Apply Acceleration
        self.vx += ax * dt
        self.vy += ay * dt
        
        # 4. Integrate Position
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # 5. Realistic Yaw/Heading control (Drones turn nose towards velocity vector)
        speed = math.hypot(self.vx, self.vy)
        if speed > 5.0:
            target_heading = math.atan2(self.vy, self.vx)
            angle_diff = constrain_angle(target_heading - self.heading)
            
            # Limit turn rate
            max_turn = MAX_TURN_RATE * dt
            if abs(angle_diff) > max_turn:
                self.heading += math.copysign(max_turn, angle_diff)
            else:
                self.heading = target_heading
                
        self.heading = constrain_angle(self.heading)
        
        # Update visual trail
        if random.random() < 0.3: # Only save some points to save memory
            self.trail.append((self.x, self.y))
            if len(self.trail) > 40:
                self.trail.pop(0)

    def draw(self, surface):
        # Draw Trail
        if len(self.trail) > 2:
            pygame.draw.aalines(surface, (80, 80, 90), False, self.trail)
            
        # Draw Target Line (Ghost line showing where the drone "wants" to be)
        pygame.draw.line(surface, (50, 50, 60), (self.x, self.y), (self.target_x, self.target_y))
        
        # Draw Drone Body
        color = LEADER_COLOR if self.node_id == 0 else DRONE_COLOR
        
        # Draw Quadcopter X Frame
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        
        # 4 Motor positions relative to center
        angles = [math.pi/4, 3*math.pi/4, -3*math.pi/4, -math.pi/4]
        for idx, a in enumerate(angles):
            # Calculate motor arm rotation
            arm_angle = self.heading + a
            mx = self.x + math.cos(arm_angle) * DRONE_SIZE
            my = self.y + math.sin(arm_angle) * DRONE_SIZE
            
            # Draw arm
            pygame.draw.line(surface, (200, 200, 200), (self.x, self.y), (mx, my), 2)
            # Draw prop/rotor (Front rotors can be marked red for orientation)
            prop_col = (255, 80, 80) if idx in (0, 3) else (150, 150, 150)
            pygame.draw.circle(surface, prop_col, (int(mx), int(my)), 4)
            
        # Center hub
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), 5)
        
        # ID text
        font = pygame.font.SysFont(None, 18)
        txt = font.render(str(self.node_id), True, (255,255,255))
        surface.blit(txt, (self.x - 4, self.y - 15))


class SwarmManager_GCS:
    """
    Simulates the Ground Control Station or central coordinator.
    It does NOT move the drones; it calculates the global virtual targets and broadcast them.
    In a real system, drones would run `calculate_local_target` locally upon receiving the broadcast.
    """
    def __init__(self):
        self.drones = []
        self.active_formation = 'V'
        
        # Global swarm state
        self.swarm_target_x = WIDTH / 2
        self.swarm_target_y = HEIGHT / 2
        self.swarm_heading = 0.0
        
        # Formations Dictionary [node_id: (forward_offset, right_offset)]
        self.formations = {
            'V': {
                0: (0, 0),        # Commander/Center
                1: (-40, -40),    # Left Wing 1
                2: (-40, 40),     # Right Wing 1
                3: (-80, -80),    # Left Wing 2
                4: (-80, 80),     # Right Wing 2
                5: (-120, -120),  # Left Wing 3
                6: (-120, 120),   # Right Wing 3
            },
            'SQUARE': {
               0: (40, 40),      # Top Right
               1: (40, -40),     # Top Left
               2: (-40, 40),     # Bottom Right
               3: (-40, -40),    # Bottom Left
               4: (0, 80),       # Far Right
               5: (0, -80),      # Far Left
               6: (-80, 0)       # Far Back
            },
            'LINE': {
                i: (-50 * i, 0) for i in range(7)
            }
        }

    def add_drone(self, drone):
        self.drones.append(drone)

    def set_formation(self, form_name):
        if form_name in self.formations:
            self.active_formation = form_name
            print(f"[GCS Network Broadcast] Formation set to: {self.active_formation}")

    def update_global_target(self, x, y):
        # Calculate heading based on direction of swarm movement
        dx = x - self.swarm_target_x
        dy = y - self.swarm_target_y
        
        if math.hypot(dx, dy) > 2.0:
            self.swarm_heading = math.atan2(dy, dx)
            
        self.swarm_target_x = x
        self.swarm_target_y = y

    def broadcast_sync(self):
        """
        Simulates the 10Hz network broadcast to all drones.
        Calculates decentralized targets.
        """
        offsets = self.formations[self.active_formation]
        
        for drone in self.drones:
            # 1. Look up offset for this specific drone's ID
            off_fwd, off_right = offsets.get(drone.node_id, (-100, 0)) # default fallback
            
            # 2. Rotate offset by global swarm heading (2D rotation matrix)
            # This ensures formation points in the direction the swarm is moving
            rot_x = off_fwd * math.cos(self.swarm_heading) - off_right * math.sin(self.swarm_heading)
            rot_y = off_fwd * math.sin(self.swarm_heading) + off_right * math.cos(self.swarm_heading)
            
            # 3. Add to global swarm target to get local target
            tx = self.swarm_target_x + rot_x
            ty = self.swarm_target_y + rot_y
            
            # 4. Transmit to flight controller
            drone.receive_target(tx, ty)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ArduPilot SITL Architecture - Real Physics Swarm")
    clock = pygame.time.Clock()

    # Create Manager and Drones
    gcs = SwarmManager_GCS()
    
    # Spawn 7 drones randomly
    for i in range(7):
        rx, ry = random.randint(100, WIDTH-100), random.randint(100, HEIGHT-100)
        d = RealisticDrone(node_id=i, x=rx, y=ry)
        gcs.add_drone(d)

    print("--- SWARM GCS ONLINE ---")
    print("Drones executing localized flight controllers.")
    
    font = pygame.font.SysFont(None, 24)
    
    # Networking sync timer
    sync_timer = 0.0
    sync_interval = 1.0 / SYNC_RATE

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        sync_timer += dt
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: gcs.set_formation('V')
                if event.key == pygame.K_2: gcs.set_formation('SQUARE')
                if event.key == pygame.K_3: gcs.set_formation('LINE')

        # GCS sets global target via Mouse
        mx, my = pygame.mouse.get_pos()
        gcs.update_global_target(mx, my)
        
        # Simulate network sync tick (e.g. 10 Hz heartbeat)
        if sync_timer >= sync_interval:
            gcs.broadcast_sync()
            sync_timer = 0.0
            
        # Drones update their internal physics loop constantly (e.g. 400Hz internally, simulated by FPS)
        for d in gcs.drones:
            d.update_physics(dt)

        # --- Rendering ---
        screen.fill(BG_COLOR)
        
        # Draw the GCS Virtual Target marker (The "Carrot" the swarm follows)
        pygame.draw.circle(screen, GCS_COLOR, (int(gcs.swarm_target_x), int(gcs.swarm_target_y)), 6, 1)
        pygame.draw.line(screen, GCS_COLOR, 
                         (gcs.swarm_target_x, gcs.swarm_target_y),
                         (gcs.swarm_target_x + math.cos(gcs.swarm_heading)*30, 
                          gcs.swarm_target_y + math.sin(gcs.swarm_heading)*30), 2)
        
        # Draw Drones
        for d in gcs.drones:
            d.draw(screen)
            
        # Draw UI
        ui_texts = [
            "REALISTIC SWARM SIMULATOR (Arch: GCS -> Edge -> Pixhawk)",
            f"Active Formation: {gcs.active_formation}",
            "Keys: [1] V-Shape  [2] Square  [3] Line",
            f"Drones Active: {len(gcs.drones)} | GCS Sync: {SYNC_RATE}Hz"
        ]
        
        for i, txt in enumerate(ui_texts):
            surf = font.render(txt, True, (200, 200, 200))
            screen.blit(surf, (15, 15 + (i * 24)))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
