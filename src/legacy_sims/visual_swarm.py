import pygame
import math
import sys
import random

# Constants
WIDTH = 800
HEIGHT = 600
FPS = 60
DRONE_RADIUS = 8
LEADER_COLOR = (255, 80, 80)
FOLLOWER_COLOR = (80, 200, 255)
BG_COLOR = (30, 30, 35)
TRAIL_COLOR = (100, 100, 110)
TRAIL_LENGTH = 30

class Drone:
    def __init__(self, x, y, is_leader=False):
        self.x = x
        self.y = y
        self.target_x = x
        self.target_y = y
        self.is_leader = is_leader
        
        # Kinematics
        self.vx = 0.0
        self.vy = 0.0
        self.max_speed = 350.0 if is_leader else 300.0  # Leader is slightly faster to pull ahead
        self.max_force = 1500.0 if is_leader else 1200.0
        self.heading = 0.0 # Radians
        self.trail = []

    def update(self, dt):
        # Steering behavior: Arrive
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        
        if dist > 0:
            speed = self.max_speed
            
            # Slow down slightly as followers approach their slot to avoid overshoot jitter
            if dist < 100 and not self.is_leader:
                speed = self.max_speed * (dist / 100.0) 
                
            desired_vx = (dx / dist) * speed
            desired_vy = (dy / dist) * speed
            
            # Steering force
            steer_x = desired_vx - self.vx
            steer_y = desired_vy - self.vy
            
            # Clamp force
            steer_mag = math.hypot(steer_x, steer_y)
            max_force = self.max_force * dt
            if steer_mag > max_force:
                steer_x = (steer_x / steer_mag) * max_force
                steer_y = (steer_y / steer_mag) * max_force
                
            self.vx += steer_x
            self.vy += steer_y
        else:
            self.vx *= 0.9 # Friction
            self.vy *= 0.9
            
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Calculate heading
        speed_sq = self.vx**2 + self.vy**2
        if speed_sq > 1.0:
            self.heading = math.atan2(self.vy, self.vx)
            
        # Update trail
        self.trail.append((self.x, self.y))
        if len(self.trail) > TRAIL_LENGTH:
            self.trail.pop(0)

    def draw(self, surface):
        if len(self.trail) > 1:
            pygame.draw.lines(surface, TRAIL_COLOR, False, self.trail, 2)
            
        color = LEADER_COLOR if self.is_leader else FOLLOWER_COLOR
        
        # Draw drone as a triangle pointing towards heading
        p1 = (self.x + math.cos(self.heading) * DRONE_RADIUS * 2, 
              self.y + math.sin(self.heading) * DRONE_RADIUS * 2)
        p2 = (self.x + math.cos(self.heading + 2.5) * DRONE_RADIUS * 1.5, 
              self.y + math.sin(self.heading + 2.5) * DRONE_RADIUS * 1.5)
        p3 = (self.x + math.cos(self.heading - 2.5) * DRONE_RADIUS * 1.5, 
              self.y + math.sin(self.heading - 2.5) * DRONE_RADIUS * 1.5)
              
        pygame.draw.polygon(surface, color, [p1, p2, p3])
        # Draw a core circle
        pygame.draw.circle(surface, (255,255,255), (int(self.x), int(self.y)), 3)


class SwarmCoordinator:
    def __init__(self, leader):
        self.leader = leader
        self.followers = []
        
        # Formations are defined by relative offsets (off_forward, off_side)
        # off_forward: negative means behind the leader
        # off_side: perpendicular to the leader's heading
        self.formations = {
            'V': [(-40, -40), (-40, 40), (-80, -80), (-80, 80), (-120, -120), (-120, 120), (-160, -160), (-160, 160)],
            'LINE': [(-50, 0), (-100, 0), (-150, 0), (-200, 0), (-250, 0), (-300, 0), (-350, 0), (-400, 0)],
            'ECHELON_RIGHT': [(-40, 40), (-80, 80), (-120, 120), (-160, 160), (-200, 200), (-240, 240), (-280, 280), (-320, 320)],
            'CIRCLE': []
        }
        
        # Dynamically generate circle formation offsets around the leader
        circle_radius = 80
        for i in range(8):
            angle = i * (2 * math.pi / 8)
            # Center the circle behind the leader slightly
            self.formations['CIRCLE'].append((-100 + math.cos(angle)*circle_radius, math.sin(angle)*circle_radius))

        self.current_formation = 'V'

    def set_formation(self, name):
        if name in self.formations:
            self.current_formation = name
            print(f"Swarm changed to '{name}' formation.")

    def add_follower(self, follower):
        self.followers.append(follower)

    def calculate_targets(self):
        offsets = self.formations[self.current_formation]
        leader_heading = self.leader.heading
        
        for i, follower in enumerate(self.followers):
            if i < len(offsets):
                off_forward, off_side = offsets[i]
                
                # Rotate offsets to align with the leader's actual heading angle!
                # This ensures the formation rotates smoothly as you turn the mouse.
                rot_x = off_forward * math.cos(leader_heading) - off_side * math.sin(leader_heading)
                rot_y = off_forward * math.sin(leader_heading) + off_side * math.cos(leader_heading)
                
                follower.target_x = self.leader.x + rot_x
                follower.target_y = self.leader.y + rot_y

    def update(self, dt):
        self.leader.update(dt)
        self.calculate_targets()
        
        for f in self.followers:
            # Soft collision avoidance between followers
            for other in self.followers:
                if f != other:
                    dx = f.x - other.x
                    dy = f.y - other.y
                    dist = math.hypot(dx, dy)
                    too_close = DRONE_RADIUS * 4
                    if 0 < dist < too_close:
                        repel_force = (too_close - dist) / too_close
                        f.x += (dx / dist) * repel_force * 2.0
                        f.y += (dy / dist) * repel_force * 2.0

            f.update(dt)

    def draw(self, surface):
        for f in reversed(self.followers): # Draw trails behind properly
            f.draw(surface)
        self.leader.draw(surface)


def main():
    try:
        pygame.init()
    except Exception as e:
        print("Failed to initialize pygame. Ensure you have a graphics environment.")
        return

    # Check for display environment
    if not pygame.display.get_init():
        print("No display environment available to open visual window.")
        return
        
    try:
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
    except pygame.error:
        print("Could not initialize video display.")
        return
        
    pygame.display.set_caption("Swarm Intelligence: Mouse Tracking Drone Simulator")
    clock = pygame.time.Clock()

    # Initialize at screen center
    leader = Drone(WIDTH//2, HEIGHT//2, is_leader=True)
    swarm = SwarmCoordinator(leader)
    
    # 8 followers spawning randomly
    print("Spawning followers...")
    for i in range(8):
        f = Drone(random.randint(0, WIDTH), random.randint(0, HEIGHT))
        swarm.add_follower(f)

    # Use default font if SysFont fails
    try:
        font = pygame.font.SysFont(None, 26)
    except:
        font = pygame.font.Font(None, 26)

    print("Simulation started. Move your mouse!")
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: swarm.set_formation('V')
                if event.key == pygame.K_2: swarm.set_formation('LINE')
                if event.key == pygame.K_3: swarm.set_formation('ECHELON_RIGHT')
                if event.key == pygame.K_4: swarm.set_formation('CIRCLE')

        # === THE MAGIC: LEAD BY MOUSE ===
        # The leader's target is continuously updated to the mouse cursor
        mouse_x, mouse_y = pygame.mouse.get_pos()
        leader.target_x = mouse_x
        leader.target_y = mouse_y

        swarm.update(dt)

        # Rendering
        screen.fill(BG_COLOR)
        swarm.draw(screen)

        # UI OVERLAY
        ui_bg = pygame.Surface((350, 110))
        ui_bg.set_alpha(150)
        ui_bg.fill((0, 0, 0))
        screen.blit(ui_bg, (10, 10))

        text_lines = [
            f"Current Formation: {swarm.current_formation}",
            "Keys: [1] V-Shape  [2] Line",
            "      [3] Echelon  [4] Circle",
            "Action: Move your mouse!"
        ]
        
        for i, line in enumerate(text_lines):
            text_surf = font.render(line, True, (220, 220, 220))
            screen.blit(text_surf, (20, 20 + i * 22))

        # Highlight target cursor
        pygame.draw.circle(screen, (255, 255, 255), (mouse_x, mouse_y), 5, 1)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
