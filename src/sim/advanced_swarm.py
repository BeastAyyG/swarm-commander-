"""
ADVANCED SWARM SIMULATION & CREATIVE RESEARCH PROOF OF CONCEPT
==============================================================

Features that set this apart from basic drone simulations:
1. Dynamic Task Allocation (The Bidding Protocol): 
   When switching formations, drones do not have hardcoded slots. 
   Instead, they run a Distributed Auction Algorithm to find the 
   optimum permutation of drone -> slot mapping that minimizes 
   the overall swarm energy expenditure (distance traveled).

2. Self-Healing Formations (Energy/Battery Simulation):
   Drones use energy based on their speed and acceleration.
   When a drone's battery drops to critically low levels, it 
   breaks formation and autonomously returns to the 'Hive' (Base Station)
   to recharge. The swarm detects the missing slot and instantly
   re-bids the remaining drones to self-heal and close the gap, 
   maintaining structural integrity.

3. Artificial Potential Fields (APF) - Obstacle Avoidance:
   The user can 'paint' obstacles (Right Click) into the environment. 
   Drones emit a localized virtual repulsive force that pushes them 
   smoothly around obstacles while their formation slot's attractive 
   force pulls them forward, causing organic fluid-like splitting.

4. Boids Flow Integration (The Free Roam Protocol):
   If the GCS releases the Swarm into 'Free Roam', they abandon
   strict formations and rely entirely on decentralized Boids 
   flocking (Cohesion, Alignment, Separation).

How to interact:
- Left Click & Drag: Move the GCS Target (Carrot)
- Right Click & Drag: Paint Obstacles (Red Circles)
- Key [1], [2], [3], [4]: Change Formations
- Key [B]: Toggle 'Free Roam' Boids Mode vs Strict Formation Mode
- Key [C]: Clear all painted obstacles
"""

import pygame
import math
import sys
import random
try:
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("\n[WARNING] Scipy not found. Falling back to greedy task allocation.")
    print("For optimal Swarm Bidding, install: python3 -m pip install numpy scipy\n")

# --- Constants ---
WIDTH, HEIGHT = 1400, 900
FPS = 60

# Physics constraints
MAX_SPEED = 300.0
MAX_ACCEL = 400.0
MAX_TURN_RATE = 5.0
OBSTACLE_REPULSION = 80000.0  # Force multiplier for obstacle avoidance
DRONE_AVOID_RADIUS = 30       # Minimum distance between drones

# Colors
BG_COLOR = (12, 16, 24)
DRONE_COLOR = (50, 220, 255)
ALERT_COLOR = (255, 60, 60)
TARGET_COLOR = (0, 255, 100)
HIVE_COLOR = (255, 200, 50)
OBSTACLE_COLOR = (255, 40, 80)
TRAIL_COLOR = (40, 60, 80)

def constrain_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class Obstacle:
    def __init__(self, x, y, radius=25):
        self.x = x
        self.y = y
        self.radius = radius

class AdvancedDrone:
    def __init__(self, drone_id, x, y):
        self.drone_id = drone_id
        
        # Physics State
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0
        
        # System State
        self.battery = random.uniform(80.0, 100.0)
        self.state = "ACTIVE" # ACTIVE, RTB (Return to Base), CHARGING
        self.assigned_slot = None # (global_x, global_y)
        
        # Visuals
        self.trail = []
        
    def consume_energy(self, dt):
        if self.state == "ACTIVE":
            speed = math.hypot(self.vx, self.vy)
            idle_drain = 0.05
            flight_drain = (speed / MAX_SPEED) * 0.2
            self.battery -= (idle_drain + flight_drain) * dt
            
            if self.battery < 20.0: # Critical threshold
                self.state = "RTB"
                print(f"[ALERT] Drone {self.drone_id} critical battery. Breaking formation for RTB.")
                self.assigned_slot = None
                
        elif self.state == "CHARGING":
            self.battery += 15.0 * dt
            if self.battery >= 100.0:
                self.battery = 100.0
                self.state = "ACTIVE"
                print(f"[INFO] Drone {self.drone_id} fully charged and requesting rejoin.")

    def calculate_apf_forces(self, obstacles, drones, dt):
        """Artificial Potential Fields: Sums up repulsive vectors."""
        repel_x, repel_y = 0.0, 0.0
        
        # Obstacle avoidance
        for obs in obstacles:
            dx = self.x - obs.x
            dy = self.y - obs.y
            dist = math.hypot(dx, dy)
            # Avoid if within influence radius
            influence = obs.radius + 60
            if 0 < dist < influence:
                force = OBSTACLE_REPULSION * (1.0/dist - 1.0/influence) / (dist**2)
                repel_x += (dx/dist) * force
                repel_y += (dy/dist) * force
                
        # Peer Drone collision avoidance (soft separation)
        for other in drones:
            if other.drone_id != self.drone_id and other.state == "ACTIVE":
                dx = self.x - other.x
                dy = self.y - other.y
                dist = math.hypot(dx, dy)
                if 0 < dist < DRONE_AVOID_RADIUS:
                    force = 5000.0 / (dist**2)
                    repel_x += (dx/dist) * force
                    repel_y += (dy/dist) * force
                    
        return repel_x, repel_y

    def update(self, dt, hive_pos, obstacles, all_drones, boids_mode):
        self.consume_energy(dt)
        
        # 1. Determine base target vector (Attraction Force)
        desired_vx, desired_vy = 0.0, 0.0
        
        if self.state == "RTB":
            # RTB logic: Fly straight to hive
            tx, ty = hive_pos
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, ty-self.y)
            if dist < 10.0:
                self.state = "CHARGING"
                self.vx, self.vy = 0, 0
            elif dist > 1.0:
                desired_vx = (dx/dist) * MAX_SPEED * 0.8 # Fly efficiently back
                desired_vy = (dy/dist) * MAX_SPEED * 0.8
                
        elif self.state == "CHARGING":
            self.x, self.y = hive_pos
            self.vx, self.vy = 0, 0
            
        elif self.state == "ACTIVE":
            if boids_mode:
                # Emergent Flocking
                desired_vx, desired_vy = self.calculate_boids(all_drones)
            else:
                # Strict Formation
                if self.assigned_slot:
                    tx, ty = self.assigned_slot
                    dx, dy = tx - self.x, ty - self.y
                    dist = math.hypot(dx, dy)
                    if dist > 2.0:
                        speed = min(MAX_SPEED, dist * 3.0) 
                        desired_vx = (dx/dist) * speed
                        desired_vy = (dy/dist) * speed
        
        # 2. Add Repulsive Forces (APF)
        if self.state != "CHARGING":
            repel_x, repel_y = self.calculate_apf_forces(obstacles, all_drones, dt)
            desired_vx += repel_x
            desired_vy += repel_y
            
        # 3. Apply Acceleration & Velocity (Kinematics)
        if self.state != "CHARGING":
            ax = (desired_vx - self.vx) * 4.0
            ay = (desired_vy - self.vy) * 4.0
            
            accel_mag = math.hypot(ax, ay)
            if accel_mag > MAX_ACCEL:
                ax = (ax / accel_mag) * MAX_ACCEL
                ay = (ay / accel_mag) * MAX_ACCEL
                
            self.vx += ax * dt
            self.vy += ay * dt
            
            # Cap max speed
            speed = math.hypot(self.vx, self.vy)
            if speed > MAX_SPEED:
                self.vx = (self.vx / speed) * MAX_SPEED
                self.vy = (self.vy / speed) * MAX_SPEED
                
            self.x += self.vx * dt
            self.y += self.vy * dt
            
            # Update Heading
            if speed > 10.0:
                target_heading = math.atan2(self.vy, self.vx)
                diff = constrain_angle(target_heading - self.heading)
                max_t = MAX_TURN_RATE * dt
                self.heading += math.copysign(min(abs(diff), max_t), diff)
                self.heading = constrain_angle(self.heading)
                
            # Trail
            if random.random() < 0.2:
                self.trail.append((self.x, self.y))
                if len(self.trail) > 25:
                    self.trail.pop(0)

    def calculate_boids(self, drones):
        """Basic Boids rules calculation"""
        com_x, com_y = 0.0, 0.0
        avg_vx, avg_vy = 0.0, 0.0
        neighbors = 0
        
        for d in drones:
            if d.drone_id != self.drone_id and d.state == "ACTIVE":
                dist = math.hypot(self.x - d.x, self.y - d.y)
                if dist < 150:
                    com_x += d.x
                    com_y += d.y
                    avg_vx += d.vx
                    avg_vy += d.vy
                    neighbors += 1
                    
        desired_vx, desired_vy = 0, 0
        if neighbors > 0:
            com_x /= neighbors
            com_y /= neighbors
            avg_vx /= neighbors
            avg_vy /= neighbors
            
            # Cohesion
            dx = com_x - self.x
            dy = com_y - self.y
            dist = math.hypot(dx, dy)
            if dist > 0:
                desired_vx += (dx/dist) * 100.0
                desired_vy += (dy/dist) * 100.0
                
            # Alignment
            desired_vx += avg_vx * 0.5
            desired_vy += avg_vy * 0.5
            
        return desired_vx, desired_vy

    def draw(self, surface):
        if self.state == "CHARGING": return # Don't draw if charging
        
        # Trail
        if len(self.trail) > 2:
            pygame.draw.aalines(surface, TRAIL_COLOR, False, self.trail)
            
        # Select Color
        if self.state == "RTB":
            color = ALERT_COLOR
        elif self.battery < 40.0:
            color = (255, 150, 50) # Orange warning
        else:
            color = DRONE_COLOR
            
        # Draw sci-fi triangle drone
        p1 = (self.x + math.cos(self.heading) * 15, self.y + math.sin(self.heading) * 15)
        p2 = (self.x + math.cos(self.heading + 2.5) * 10, self.y + math.sin(self.heading + 2.5) * 10)
        p3 = (self.x + math.cos(self.heading - 2.5) * 10, self.y + math.sin(self.heading - 2.5) * 10)
        
        pygame.draw.polygon(surface, color, [p1, p2, p3])
        
        # Battery Bar
        bar_w = 20
        fill_w = int((self.battery / 100.0) * bar_w)
        pygame.draw.rect(surface, (100, 100, 100), (self.x - 10, self.y - 20, bar_w, 4))
        pygame.draw.rect(surface, color, (self.x - 10, self.y - 20, fill_w, 4))

class AdvancedSwarmGCS:
    def __init__(self):
        self.drones = []
        self.obstacles = []
        self.hive_pos = (WIDTH//2, HEIGHT - 50)
        
        self.target_x = WIDTH//2
        self.target_y = HEIGHT//2
        self.heading = -math.pi/2 # Pointing UP initially
        
        self.formation = 'V'
        self.boids_mode = False
        self.active_slots = [] # List of (global_x, global_y) currently calculated
        
        # Defines the mathematical structure of the formations relative to 0,0
        # Multiplier scales the distance between drones
        self.spacing = 60
        self.formation_blueprints = {
            'V': [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2), (-3,-3), (-3,3), (-4,-4), (-4,4), (-5,-5), (-5,5), (-6,-6), (-6,6)],
            'ARROW': [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2), (-2,0), (-3,0), (-4,0), (-5,0), (-6,0)],
            'HEXAGON': [(0,0), (1,1), (1,-1), (2,0), (-1,1), (-1,-1), (-2,0), (0,2), (0,-2), (2,2), (2,-2), (-2,2), (-2,-2)],
            'WALL': [(0,i) for i in range(-6, 7)]
        }

    def allocate_tasks_auction(self):
        """
        The Bidding Protocol!
        Instead of Drone 1 always going to Point A, we map N drones to M slots
        such that the total flight distance of the *entire swarm* is minimized.
        This enables beautiful self-healing and morphing.
        """
        active_drones = [d for d in self.drones if d.state == "ACTIVE"]
        if not active_drones or self.boids_mode:
            return
            
        # 1. Calculate the required number of global slots for the current active fleet
        n_active = len(active_drones)
        blueprint = self.formation_blueprints[self.formation]
        # We only take as many slots as we have drones (Self Healing!)
        slots_needed = blueprint[:min(n_active, len(blueprint))]
        
        # 2. Translate blueprint into global rotated coordinates based on Swarm GCS
        global_slots = []
        for (fwd, right) in slots_needed:
            # Mathematical rotation
            rx = fwd * self.spacing * math.cos(self.heading) - right * self.spacing * math.sin(self.heading)
            ry = fwd * self.spacing * math.sin(self.heading) + right * self.spacing * math.cos(self.heading)
            global_slots.append((self.target_x + rx, self.target_y + ry))
            
        self.active_slots = global_slots # Store for visualization
        
        # 3. Cost Matrix (Distance from each drone to each slot)
        # Using Hungarian Algorithm via scipy, or greedy fallback
        if not global_slots: return
        
        if HAS_SCIPY:
            cost_matrix = np.zeros((len(active_drones), len(global_slots)))
            for i, drone in enumerate(active_drones):
                for j, slot in enumerate(global_slots):
                    # Squared distance is a great cost function for energy minimization
                    cost_matrix[i, j] = (drone.x - slot[0])**2 + (drone.y - slot[1])**2
                    
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            for i, j in zip(row_ind, col_ind):
                active_drones[i].assigned_slot = global_slots[j]
        else:
            # Greedy allocation for environments without scipy
            unassigned_drones = active_drones.copy()
            unassigned_slots = global_slots.copy()
            
            while unassigned_drones and unassigned_slots:
                best_pair = None
                best_dist = float('inf')
                
                for d in unassigned_drones:
                    for s in unassigned_slots:
                        dist = math.hypot(d.x - s[0], d.y - s[1])
                        if dist < best_dist:
                            best_dist = dist
                            best_pair = (d, s)
                            
                d, s = best_pair
                d.assigned_slot = s
                unassigned_drones.remove(d)
                unassigned_slots.remove(s)

    def paint_obstacle(self, x, y):
        # Don't spawn them overlapping too much
        for obs in self.obstacles:
            if math.hypot(obs.x - x, obs.y - y) < 30:
                return
        self.obstacles.append(Obstacle(x, y, radius=random.randint(20, 50)))

def text_glow(surface, text, font, pos, color):
    """Render text with a slight bloom/glow effect."""
    text_surf = font.render(text, True, color)
    glow_surf = font.render(text, True, (color[0]//2, color[1]//2, color[2]//2))
    surface.blit(glow_surf, (pos[0]+2, pos[1]+2)) # Fake shadow/glow
    surface.blit(text_surf, pos)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Next-Gen Autonomous Swarm Architecture")
    clock = pygame.time.Clock()

    gcs = AdvancedSwarmGCS()
    
    # Spawn a massive swarm of 12 drones
    for i in range(12):
        d = AdvancedDrone(i, WIDTH//2 + random.uniform(-100, 100), HEIGHT - 100)
        gcs.drones.append(d)

    ui_font = pygame.font.SysFont("Courier", 20, bold=True)
    title_font = pygame.font.SysFont("Courier", 28, bold=True)
    
    # To track when we need to re-run the auction (when swarm shape/state changes)
    last_active_count = 0
    auction_timer = 0.0

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        auction_timer += dt
        
        mouse_x, mouse_y = pygame.mouse.get_pos()
        buttons = pygame.mouse.get_pressed()
        
        # Left click to set GCS target
        if buttons[0]:
            gcs.target_x = mouse_x
            gcs.target_y = mouse_y
            
        # Right click to paint obstacles
        if buttons[2]:
            gcs.paint_obstacle(mouse_x, mouse_y)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                trigger_auction = False
                if event.key == pygame.K_1: gcs.formation, trigger_auction = 'V', True
                if event.key == pygame.K_2: gcs.formation, trigger_auction = 'ARROW', True
                if event.key == pygame.K_3: gcs.formation, trigger_auction = 'HEXAGON', True
                if event.key == pygame.K_4: gcs.formation, trigger_auction = 'WALL', True
                
                if event.key == pygame.K_b:
                    gcs.boids_mode = not gcs.boids_mode
                    trigger_auction = True
                    
                if event.key == pygame.K_c:
                    gcs.obstacles.clear()
                    
                if trigger_auction: 
                    gcs.allocate_tasks_auction()

        # Update Swarm Heading to face target
        com_x = sum([d.x for d in gcs.drones]) / len(gcs.drones)
        com_y = sum([d.y for d in gcs.drones]) / len(gcs.drones)
        dx = gcs.target_x - com_x
        dy = gcs.target_y - com_y
        if math.hypot(dx, dy) > 10.0:
            # Smooth rotate heading
            target_heading = math.atan2(dy, dx)
            diff = constrain_angle(target_heading - gcs.heading)
            gcs.heading += diff * 0.1 # Exponential smoothing factor

        # Re-run auction periodically or if a drone dies (Self-Healing)
        active_count = len([d for d in gcs.drones if d.state == "ACTIVE"])
        if active_count != last_active_count or auction_timer > 0.2: # 5Hz re-allocation
            gcs.allocate_tasks_auction()
            auction_timer = 0.0
            last_active_count = active_count

        # Update Phsyics
        for d in gcs.drones:
            d.update(dt, gcs.hive_pos, gcs.obstacles, gcs.drones, gcs.boids_mode)

        # --- RENDERING ---
        screen.fill(BG_COLOR)
        
        # Draw Hive base
        pygame.draw.rect(screen, HIVE_COLOR, (WIDTH//2 - 40, HEIGHT - 20, 80, 20))
        text_glow(screen, "HIVE / CHARGING PAD", pygame.font.SysFont("Courier", 14), (WIDTH//2 - 70, HEIGHT - 40), HIVE_COLOR)
        
        # Draw Obstacles (APF Fields)
        for obs in gcs.obstacles:
            # Core
            pygame.draw.circle(screen, OBSTACLE_COLOR, (int(obs.x), int(obs.y)), obs.radius)
            # Area of influence (Field)
            pygame.draw.circle(screen, (80, 30, 40), (int(obs.x), int(obs.y)), obs.radius + 60, 2)
            
        # Draw GCS Target
        if not gcs.boids_mode:
            pygame.draw.circle(screen, TARGET_COLOR, (int(gcs.target_x), int(gcs.target_y)), 10, 2)
            pygame.draw.line(screen, TARGET_COLOR, 
                            (gcs.target_x, gcs.target_y), 
                            (gcs.target_x + math.cos(gcs.heading)*40, gcs.target_y + math.sin(gcs.heading)*40), 2)
                            
            # Draw Holographic Formation Slots (Where the drones are bidding to go)
            for slot in gcs.active_slots:
                # Ghostly rings
                pygame.draw.circle(screen, (30, 80, 60), (int(slot[0]), int(slot[1])), 8, 1)

        # Draw Drones
        for d in gcs.drones:
            d.draw(screen)

        # Draw UI Overlay
        ui_bg = pygame.Surface((400, 250))
        ui_bg.set_alpha(200)
        ui_bg.fill((5, 10, 15))
        screen.blit(ui_bg, (20, 20))

        text_glow(screen, "NEXT-GEN SWARM PROTOCOL", title_font, (30, 30), (100, 200, 255))
        
        mode_str = "BOIDS (FREE FLOCK)" if gcs.boids_mode else f"STRICT: {gcs.formation}"
        texts = [
            f"Mode: {mode_str}",
            f"Active Fleet:  {active_count}/{len(gcs.drones)}",
            f"Charging/RTB:  {len(gcs.drones) - active_count}",
            "",
            "[L-Click] Move GCS Target",
            "[R-Click] Paint Obstacles (APF Field)",
            "[1]-[4]   Morph Formations",
            "[B]       Toggle Boids Flow Mode",
            "[C]       Clear Obstacles"
        ]
        
        for i, txt in enumerate(texts):
            color = (200, 200, 200)
            if "Mode:" in txt: color = (255, 200, 100)
            text_glow(screen, txt, ui_font, (30, 70 + (i * 22)), color)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
