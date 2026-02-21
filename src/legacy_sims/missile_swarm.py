"""
2D TOP-DOWN SWARM MISSILE EVASION SIMULATION
============================================

Showcases Swarm scattering under Artificial Potential Fields (APF)
to dynamically dodge incoming threats (Missiles) and then self-heal
back into formation.

Features:
- Left Click: Move Swarm Target
- Right Click: Launch Anti-Air Missile from that location towards swarm
- Press [1], [2], [3], [4]: Change Swarm Formation
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

# --- CONFIGURATION ---
WIDTH, HEIGHT = 1400, 900
FPS = 60

# Physics
DRONE_MAX_SPEED = 250.0
DRONE_MAX_ACCEL = 400.0
DRONE_MAX_TURN = 5.0
MISSILE_SPEED = 600.0
MISSILE_TURN_RATE = 2.0  # Slightly sluggish turning so they can be dodged

# APF Settings (The "Dodging" mechanism)
MISSILE_REPULSION_FORCE = 5000000.0  # Massive force pushing drones away
MISSILE_INFLUENCE_RADIUS = 300.0     # Drones start dodging when missile is this close
DRONE_REPULSION_FORCE = 8000.0       # Keeps drones from crashing into each other

# Colors
COLOR_BG = (10, 15, 20)
COLOR_DRONE = (0, 255, 150)
COLOR_DRONE_TRAIL = (0, 80, 50)
COLOR_MISSILE = (255, 50, 50)
COLOR_MISSILE_TRAIL = (255, 100, 0)
COLOR_TARGET = (100, 150, 255)
COLOR_RADAR = (0, 50, 20)
COLOR_EXPLOSION = (255, 200, 100)

def constrain_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class Missile:
    def __init__(self, x, y, target_x, target_y):
        self.x = float(x)
        self.y = float(y)
        
        # Point towards target initially
        dx = target_x - x
        dy = target_y - y
        self.heading = math.atan2(dy, dx)
        
        self.vx = math.cos(self.heading) * MISSILE_SPEED
        self.vy = math.sin(self.heading) * MISSILE_SPEED
        
        self.active = True
        self.trail = []
        
        # Explosion Tracking
        self.exploded = False
        self.explosion_radius = 80.0
        self.explosion_timer = 0.0

    def update(self, dt, swarm_center):
        if self.exploded:
            self.explosion_timer += dt
            if self.explosion_timer > 0.5:
                self.active = False
            return

        # Proportional Navigation (Seek the swarm center)
        cx, cy = swarm_center
        dx = cx - self.x
        dy = cy - self.y
        dist = math.hypot(dx, dy)
        
        if dist < 40.0:
            # Detonate!
            self.exploded = True
            return

        # Constrained turning (Missiles can't turn instantly)
        desired_heading = math.atan2(dy, dx)
        diff = constrain_angle(desired_heading - self.heading)
        
        max_turn = MISSILE_TURN_RATE * dt
        self.heading += math.copysign(min(abs(diff), max_turn), diff)
        self.heading = constrain_angle(self.heading)
        
        # Move
        self.vx = math.cos(self.heading) * MISSILE_SPEED
        self.vy = math.sin(self.heading) * MISSILE_SPEED
        
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        self.trail.append((self.x, self.y))
        if len(self.trail) > 30:
            self.trail.pop(0)

    def draw(self, surface):
        if self.exploded:
            # Draw expanding explosion
            progress = self.explosion_timer / 0.5
            current_radius = int(self.explosion_radius * math.sqrt(progress))
            alpha = max(0, int(255 * (1.0 - progress)))
            
            # Pygame doesn't support alpha circles easily without surfaces, so we make a surface
            exp_surf = pygame.Surface((current_radius*2, current_radius*2), pygame.SRCALPHA)
            pygame.draw.circle(exp_surf, (255, 200, 100, alpha), (current_radius, current_radius), current_radius)
            pygame.draw.circle(exp_surf, (255, 100, 50, alpha), (current_radius, current_radius), current_radius*0.7)
            surface.blit(exp_surf, (int(self.x - current_radius), int(self.y - current_radius)))
            return

        # Trail
        if len(self.trail) > 2:
            pygame.draw.lines(surface, COLOR_MISSILE_TRAIL, False, self.trail, 3)
            
        # Missile Body
        p1 = (self.x + math.cos(self.heading) * 15, self.y + math.sin(self.heading) * 15)
        p2 = (self.x + math.cos(self.heading + 2) * 5, self.y + math.sin(self.heading + 2) * 5)
        p3 = (self.x + math.cos(self.heading - 2) * 5, self.y + math.sin(self.heading - 2) * 5)
        pygame.draw.polygon(surface, COLOR_MISSILE, [p1, p2, p3])

class Drone:
    def __init__(self, drone_id, x, y):
        self.drone_id = drone_id
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0
        self.assigned_slot = None
        self.active = True
        self.trail = []

    def update(self, dt, drones, missiles):
        if not self.active: return
        
        # 1. Base Target Attraction Force
        desired_vx, desired_vy = 0.0, 0.0
        
        if self.assigned_slot:
            tx, ty = self.assigned_slot
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist > 2.0:
                speed = min(DRONE_MAX_SPEED, dist * 3.0) 
                desired_vx = (dx/dist) * speed
                desired_vy = (dy/dist) * speed
                
        # 2. Artificial Potential Fields (Repulsion)
        repel_x, repel_y = 0.0, 0.0
        
        # Drone-To-Drone Collision Avoidance
        for other in drones:
            if other.drone_id != self.drone_id and other.active:
                dx = self.x - other.x
                dy = self.y - other.y
                dist = max(0.1, math.hypot(dx, dy))
                if dist < 40.0:
                    force = DRONE_REPULSION_FORCE / (dist**2)
                    repel_x += (dx/dist) * force
                    repel_y += (dy/dist) * force
                    
        # Missile Dodging APF (The core logic!)
        for m in missiles:
            if not m.exploded:
                dx = self.x - m.x
                dy = self.y - m.y
                dist = max(0.1, math.hypot(dx, dy))
                
                # If missile is within the sphere of influence, PANIC dive out of the way
                if dist < MISSILE_INFLUENCE_RADIUS:
                    # Exponential repel force: gets dramatically stronger as missile gets closer
                    force = MISSILE_REPULSION_FORCE * (1.0/dist - 1.0/MISSILE_INFLUENCE_RADIUS) / (dist**2)
                    repel_x += (dx/dist) * force
                    repel_y += (dy/dist) * force
            else:
                # Check for death radius if explosion triggered
                if math.hypot(self.x - m.x, self.y - m.y) < m.explosion_radius:
                    self.active = False
                    return
                    
        # Add APF forces to desired velocity
        desired_vx += repel_x
        desired_vy += repel_y
        
        # 3. Apply Kinematics
        ax = (desired_vx - self.vx) * 4.0
        ay = (desired_vy - self.vy) * 4.0
        
        accel_mag = math.hypot(ax, ay)
        if accel_mag > DRONE_MAX_ACCEL:
            ax = (ax / accel_mag) * DRONE_MAX_ACCEL
            ay = (ay / accel_mag) * DRONE_MAX_ACCEL
            
        self.vx += ax * dt
        self.vy += ay * dt
        
        # Cap Speed
        speed = math.hypot(self.vx, self.vy)
        # Allow drones to overspeed slightly when dodging for their lives
        limit = DRONE_MAX_SPEED * 1.5 if (abs(repel_x) > 100 or abs(repel_y) > 100) else DRONE_MAX_SPEED
        if speed > limit:
            self.vx = (self.vx / speed) * limit
            self.vy = (self.vy / speed) * limit
            
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Update Heading visually
        if speed > 5.0:
            target_heading = math.atan2(self.vy, self.vx)
            diff = constrain_angle(target_heading - self.heading)
            # Allow faster turning when dodging
            turn_limit = DRONE_MAX_TURN * 2.0 * dt
            self.heading += math.copysign(min(abs(diff), turn_limit), diff)
            self.heading = constrain_angle(self.heading)
            
        # Add visual trail
        if random.random() < 0.2:
            self.trail.append((self.x, self.y))
            if len(self.trail) > 15:
                self.trail.pop(0)

    def draw(self, surface):
        if not self.active: return
        
        if len(self.trail) > 2:
            pygame.draw.lines(surface, COLOR_DRONE_TRAIL, False, self.trail, 2)
            
        p1 = (self.x + math.cos(self.heading) * 12, self.y + math.sin(self.heading) * 12)
        p2 = (self.x + math.cos(self.heading + 2.4) * 8, self.y + math.sin(self.heading + 2.4) * 8)
        p3 = (self.x + math.cos(self.heading - 2.4) * 8, self.y + math.sin(self.heading - 2.4) * 8)
        
        pygame.draw.polygon(surface, COLOR_DRONE, [p1, p2, p3])

class SwarmController:
    def __init__(self, target_x, target_y):
        self.drones = []
        self.missiles = []
        
        self.target_x = target_x
        self.target_y = target_y
        self.heading = -math.pi/2
        
        self.formation = 'V'
        self.spacing = 50.0
        
        self.blueprints = {
             'V': [(0,0), (-1,-1), (-1,1), (-2,-2), (-2,2), (-3,-3), (-3,3), (-4,-4), (-4,4), (-5,-5), (-5,5)],
             'HEXAGON': [(0,0), (1,1), (1,-1), (2,0), (-1,1), (-1,-1), (-2,0), (0,2), (0,-2), (2,2), (2,-2), (-2,2), (-2,-2)],
             'CIRCLE': [(math.cos(i*math.pi/4)*1.5, math.sin(i*math.pi/4)*1.5) for i in range(8)] + [(0,0)],
             'LINE': [(0, i) for i in range(-5, 6)]
        }
        
    def add_drone(self, d):
        self.drones.append(d)

    def fire_missile(self, start_x, start_y):
        # Target the center of mass of the alive swarm
        alive = [d for d in self.drones if d.active]
        if not alive: return
        
        cx = sum(d.x for d in alive) / len(alive)
        cy = sum(d.y for d in alive) / len(alive)
        
        self.missiles.append(Missile(start_x, start_y, cx, cy))

    def run_bidding(self):
        """Self-Healing allocation. Always assigns available slots to surviving drones."""
        alive_drones = [d for d in self.drones if d.active]
        if not alive_drones: return
        
        blueprint = self.blueprints[self.formation]
        slots_needed = blueprint[:min(len(alive_drones), len(blueprint))]
        
        # Calculate Global Slot Coordinates
        global_slots = []
        for (fwd, right) in slots_needed:
            # Rotate
            rx = fwd * self.spacing * math.cos(self.heading) - right * self.spacing * math.sin(self.heading)
            ry = fwd * self.spacing * math.sin(self.heading) + right * self.spacing * math.cos(self.heading)
            global_slots.append((self.target_x + rx, self.target_y + ry))

        # Assign slots
        if HAS_SCIPY and len(alive_drones) <= len(global_slots):
            # Optimal Bidding
            cost_matrix = np.zeros((len(alive_drones), len(global_slots)))
            for i, d in enumerate(alive_drones):
                for j, s in enumerate(global_slots):
                    cost_matrix[i, j] = (d.x - s[0])**2 + (d.y - s[1])**2
            
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for i, j in zip(row_ind, col_ind):
                alive_drones[i].assigned_slot = global_slots[j]
        else:
            # Greedy backup
            for idx, d in enumerate(alive_drones):
                if idx < len(global_slots):
                    d.assigned_slot = global_slots[idx]

    def update(self, dt):
        alive_drones = [d for d in self.drones if d.active]
        if not alive_drones: return
        
        # Orient formation towards target
        com_x = sum(d.x for d in alive_drones) / len(alive_drones)
        com_y = sum(d.y for d in alive_drones) / len(alive_drones)
        
        dx = self.target_x - com_x
        dy = self.target_y - com_y
        
        if math.hypot(dx, dy) > 10.0:
            target_h = math.atan2(dy, dx)
            diff = constrain_angle(target_h - self.heading)
            self.heading += diff * 0.1 # Exponential smoothing
            
        # Re-run bidding every frame to handle dynamic destruction and rotation seamlessly
        self.run_bidding()
        
        # Update Kinetics
        for m in reversed(self.missiles):
            m.update(dt, (com_x, com_y))
            if not m.active:
                self.missiles.remove(m)
                
        for d in self.drones:
            d.update(dt, self.drones, self.missiles)

def draw_ui(screen, font, controller, num_total):
    num_alive = len([d for d in controller.drones if d.active])
    
    # Radar Sweep effect
    t = pygame.time.get_ticks() / 1000.0
    sweep_angle = t * 2.0
    radar_center = (WIDTH - 150, HEIGHT - 150)
    pygame.draw.circle(screen, COLOR_RADAR, radar_center, 120, 1)
    
    r_x = radar_center[0] + math.cos(sweep_angle) * 120
    r_y = radar_center[1] + math.sin(sweep_angle) * 120
    pygame.draw.line(screen, (0, 200, 50), radar_center, (r_x, r_y), 2)
    
    # Mini-map blips
    scale = 120.0 / (max(WIDTH, HEIGHT)/1.5)
    for m in controller.missiles:
        if not m.exploded:
            bx = radar_center[0] + (m.x - WIDTH/2) * scale
            by = radar_center[1] + (m.y - HEIGHT/2) * scale
            pygame.draw.circle(screen, (255, 0, 0), (int(bx), int(by)), 3)

    ui_bg = pygame.Surface((380, 200))
    ui_bg.set_alpha(200)
    ui_bg.fill((0,0,0))
    screen.blit(ui_bg, (20, 20))

    texts = [
        "ADVANCED THREAT EVASION SYSTEM (APF)",
        f"Active Formation:  {controller.formation}",
        f"Swarm Integrity:   {num_alive} / {num_total}",
        f"Active Threats:    {len([m for m in controller.missiles if not m.exploded])}",
        "",
        "[L-Click] Move Swarm Waypoint",
        "[R-Click] Launch Anti-Air Missile",
        "KEYS 1-4: Morph Formation Patterns"
    ]

    for i, line in enumerate(texts):
        col = (255, 100, 100) if "Threat" in line and controller.missiles else (200, 200, 200)
        col = (0, 255, 150) if "Integrity" in line else col
        s = font.render(line, True, col)
        screen.blit(s, (35, 30 + i * 20))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Swarm Missile Evasion (Artificial Potential Fields)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Courier", 16, bold=True)

    swarm = SwarmController(WIDTH//2, HEIGHT//2)
    
    TOTAL_DRONES = 11
    print("Spawning Swarm Fleet...")
    for i in range(TOTAL_DRONES):
        swarm.add_drone(Drone(i, WIDTH//2 + random.uniform(-100, 100), HEIGHT//2 + random.uniform(-100, 100)))

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        mx, my = pygame.mouse.get_pos()
        buttons = pygame.mouse.get_pressed()
        
        # Left Click -> Move Swarm
        if buttons[0]:
            swarm.target_x = mx
            swarm.target_y = my

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: swarm.formation = 'V'
                if event.key == pygame.K_2: swarm.formation = 'HEXAGON'
                if event.key == pygame.K_3: swarm.formation = 'CIRCLE'
                if event.key == pygame.K_4: swarm.formation = 'LINE'
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3: # Right Click
                    swarm.fire_missile(mx, my)

        # Logic
        swarm.update(dt)

        # Render
        screen.fill(COLOR_BG)
        
        # Draw Target Crosshair
        pygame.draw.circle(screen, COLOR_TARGET, (int(swarm.target_x), int(swarm.target_y)), 15, 1)
        pygame.draw.line(screen, COLOR_TARGET, (swarm.target_x-20, swarm.target_y), (swarm.target_x+20, swarm.target_y), 1)
        pygame.draw.line(screen, COLOR_TARGET, (swarm.target_x, swarm.target_y-20), (swarm.target_x, swarm.target_y+20), 1)

        # Draw Entities
        for d in swarm.drones:
            d.draw(screen)
            
        for m in swarm.missiles:
            m.draw(screen)
            
        draw_ui(screen, font, swarm, TOTAL_DRONES)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
