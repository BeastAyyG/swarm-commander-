#!/usr/bin/env python3
"""
inspection_demo.py — Visual Structural Inspection Simulation
==============================================================
Standalone Pygame demo showing multiple drones performing a coordinated
structural inspection. Each drone orbits its assigned sector of the
structure at descending altitudes, simulating façade scanning.

No ArduPilot required — pure simulation for demonstration.

Controls:
  Left-Click: Place structure to inspect
  Right-Click: Place obstacle
  1-7: Change number of drones
  +/-: Adjust orbit radius
  SPACE: Start/pause inspection
  R: Reset mission
  ESC: Quit
"""

import sys, os, math, time, random
import pygame

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from structural_inspection import (
    Structure, InspectionConfig, StructuralInspectionPlanner,
    InspectionPhase, ScanPoint
)

# ==================== CONFIG ====================
WIDTH, HEIGHT = 1400, 900
FPS = 60
MPP = 0.35  # meters per pixel

# Colors
BG = (8, 10, 18)
GRID = (18, 22, 32)
GRID_MAJ = (28, 35, 50)
STRUCT_COLOR = (60, 70, 90)
STRUCT_OUTLINE = (100, 120, 150)
SECTOR_LINE = (50, 60, 80)
SCAN_DONE = (40, 180, 80, 80)
SCAN_PENDING = (60, 60, 80, 60)
SCAN_ACTIVE = (255, 220, 50)
ORBIT_PATH = (40, 50, 70)
HOME_COLOR = (255, 230, 80)
TEXT = (200, 210, 225)
DIM = (90, 100, 120)
BRIGHT = (240, 245, 255)
PANEL_BG = (12, 14, 22, 220)

DRONE_COLORS = [
    (0, 200, 255), (255, 130, 40), (80, 255, 130),
    (255, 220, 50), (190, 100, 255), (255, 80, 170), (100, 230, 210),
]

# ==================== SIMULATED DRONE ====================
class SimDrone:
    def __init__(self, did, color, hx=0, hy=0):
        self.id = did
        self.color = color
        self.x, self.y, self.alt = hx, hy, 0.0
        self.target_x, self.target_y, self.target_alt = hx, hy, 0.0
        self.heading = 0.0
        self.speed = 5.0
        self.trail = []
        self.home_x, self.home_y = hx, hy
        self.active = False
        self.photos = 0
        self.sector = -1
        self.status = "Idle"

    def move_toward_target(self, dt):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        da = self.target_alt - self.alt
        dist = math.sqrt(dx*dx + dy*dy + da*da)
        if dist < 0.5:
            return True  # Arrived
        step = min(self.speed * dt, dist)
        self.x += dx / dist * step
        self.y += dy / dist * step
        self.alt += da / dist * step
        # Face toward structure
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            self.heading = math.atan2(dy, dx)
        # Trail
        self.trail.append((self.x, self.y, time.time()))
        if len(self.trail) > 800:
            self.trail.pop(0)
        return False

# ==================== CAMERA ====================
class Camera:
    def __init__(self):
        self.x, self.y = 0.0, 0.0
        self.mpp = MPP
    def w2s(self, mx, my):
        return int(WIDTH//2 + (mx-self.x)/self.mpp), int(HEIGHT//2 - (my-self.y)/self.mpp)
    def s2w(self, sx, sy):
        return (sx-WIDTH//2)*self.mpp + self.x, -(sy-HEIGHT//2)*self.mpp + self.y
    def zoom(self, d):
        self.mpp = max(0.08, min(2.0, self.mpp - d*0.03))

# ==================== DRAWING ====================
def draw_grid(surf, cam):
    step = 10 if cam.mpp < 0.6 else 20 if cam.mpp < 1.0 else 50
    for g in range(-500, 501, step):
        sx, _ = cam.w2s(g, 0)
        if 0 <= sx <= WIDTH:
            pygame.draw.line(surf, GRID_MAJ if g%50==0 else GRID, (sx,0), (sx,HEIGHT), 1)
        _, sy = cam.w2s(0, g)
        if 0 <= sy <= HEIGHT:
            pygame.draw.line(surf, GRID_MAJ if g%50==0 else GRID, (0,sy), (WIDTH,sy), 1)

def draw_structure(surf, cam, struct):
    sx, sy = cam.w2s(struct.center_x, struct.center_y)
    pr = int(struct.radius / cam.mpp)
    # Shadow
    shadow = pygame.Surface((pr*2+20, pr*2+20), pygame.SRCALPHA)
    pygame.draw.circle(shadow, (0,0,0,40), (pr+10, pr+10), pr+8)
    surf.blit(shadow, (sx-pr-10, sy-pr-10))
    # Structure body
    pygame.draw.circle(surf, STRUCT_COLOR, (sx, sy), pr)
    pygame.draw.circle(surf, STRUCT_OUTLINE, (sx, sy), pr, 2)
    # Cross
    pygame.draw.line(surf, STRUCT_OUTLINE, (sx-pr//2, sy), (sx+pr//2, sy), 1)
    pygame.draw.line(surf, STRUCT_OUTLINE, (sx, sy-pr//2), (sx, sy+pr//2), 1)

def draw_orbit_path(surf, cam, struct, config):
    sx, sy = cam.w2s(struct.center_x, struct.center_y)
    or_px = int(config.orbit_radius / cam.mpp)
    pygame.draw.circle(surf, ORBIT_PATH, (sx, sy), or_px, 1)

def draw_sectors(surf, cam, planner):
    lines = planner.get_sector_boundaries()
    for x1, y1, x2, y2 in lines:
        s1 = cam.w2s(x1, y1)
        s2 = cam.w2s(x2, y2)
        pygame.draw.line(surf, SECTOR_LINE, s1, s2, 1)

def draw_scan_points(surf, cam, planner, font):
    for a in planner.assignments:
        color = DRONE_COLORS[a.drone_id - 1 % len(DRONE_COLORS)]
        for i, pt in enumerate(a.scan_points):
            sx, sy = cam.w2s(pt.x, pt.y)
            if 0 <= sx <= WIDTH and 0 <= sy <= HEIGHT:
                if i < a.current_point_idx:
                    # Done
                    pygame.draw.circle(surf, (*color[:3], 60) if len(color)==3 else color, (sx,sy), 3)
                    pygame.draw.circle(surf, (40,180,80), (sx,sy), 2)
                elif i == a.current_point_idx:
                    # Active
                    pulse = abs(math.sin(time.time()*5)) * 4 + 5
                    pygame.draw.circle(surf, SCAN_ACTIVE, (sx,sy), int(pulse), 2)
                else:
                    # Pending
                    pygame.draw.circle(surf, (50,55,70), (sx,sy), 2)

def draw_drone(surf, cam, drone, font, t):
    sx, sy = cam.w2s(drone.x, drone.y)
    # Trail
    if len(drone.trail) > 2:
        ts = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        for i in range(1, len(drone.trail)):
            age = t - drone.trail[i][2]
            if age > 20: continue
            alpha = int(max(10, 80 - age*4))
            p1 = cam.w2s(drone.trail[i-1][0], drone.trail[i-1][1])
            p2 = cam.w2s(drone.trail[i][0], drone.trail[i][1])
            pygame.draw.line(ts, (*drone.color, alpha), p1, p2, 2)
        surf.blit(ts, (0,0))

    # Body
    size = max(8, int(14 / max(cam.mpp, 0.2)))
    h = drone.heading
    # Arms + spinning props
    pt = t * 12 + drone.id
    for ao in [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]:
        ex = sx + int(size * math.cos(h + ao))
        ey = sy - int(size * math.sin(h + ao))
        pygame.draw.line(surf, drone.color, (sx,sy), (ex,ey), 2)
        for b in range(3):
            bx = ex + int(4*math.cos(pt + b*2.094 + ao))
            by = ey - int(4*math.sin(pt + b*2.094 + ao))
            pygame.draw.line(surf, drone.color, (ex,ey), (bx,by), 1)
    pygame.draw.circle(surf, BRIGHT, (sx,sy), 3)
    # Nose
    nx = sx + int(size*1.3*math.cos(h+math.pi/2))
    ny = sy - int(size*1.3*math.sin(h+math.pi/2))
    pygame.draw.line(surf, (255,80,80), (sx,sy), (nx,ny), 2)
    # Label
    surf.blit(font.render(f"D{drone.id}", True, drone.color), (sx+18, sy-10))
    surf.blit(font.render(f"{drone.alt:.0f}m", True, DIM), (sx+18, sy+4))
    # Camera FOV indicator (cone toward structure)
    if drone.active:
        fov = math.pi / 6
        fl = 20
        for side in [-fov/2, fov/2]:
            fx = sx + int(fl * math.cos(drone.heading + math.pi + side))
            fy = sy - int(fl * math.sin(drone.heading + math.pi + side))
            pygame.draw.line(surf, (*drone.color, 80) if len(drone.color)==3 else drone.color,
                           (sx,sy), (fx,fy), 1)

def draw_hud(surf, planner, drones, elapsed, font_s, font_l, num_drones):
    # Top bar
    tb = pygame.Surface((WIDTH, 55), pygame.SRCALPHA); tb.fill(PANEL_BG)
    surf.blit(tb, (0,0))
    surf.blit(font_l.render("STRUCTURAL INSPECTION — Autonomous Multi-Drone Scanner", True, BRIGHT), (20,5))
    phase_color = (60,255,130) if planner.phase == InspectionPhase.COMPLETE else \
                  SCAN_ACTIVE if planner.phase == InspectionPhase.SCANNING else DIM
    surf.blit(font_l.render(f"▸ {planner.phase.value} — {planner.overall_progress:.0f}% complete",
                            True, phase_color), (20, 28))
    surf.blit(font_l.render(f"T+{int(elapsed)}s  |  {num_drones} Drones", True, DIM),
              (WIDTH-250, 5))

    # Progress bar
    pw = 300
    pygame.draw.rect(surf, (30,35,50), (WIDTH-pw-20, 30, pw, 12), border_radius=4)
    fill = int(pw * planner.overall_progress / 100)
    if fill > 0:
        pygame.draw.rect(surf, phase_color, (WIDTH-pw-20, 30, fill, 12), border_radius=4)

    # Right panel — per-drone status
    rw = 290
    rh = 25 + len(drones) * 62
    panel = pygame.Surface((rw, rh), pygame.SRCALPHA); panel.fill(PANEL_BG)
    surf.blit(panel, (WIDTH-rw-10, 65))
    y = 72
    surf.blit(font_s.render("━━ INSPECTION STATUS ━━", True, DIM), (WIDTH-rw, y)); y += 18

    status = planner.get_status_summary()
    for ds in status['drones']:
        did = ds['id']
        dc = DRONE_COLORS[(did-1) % len(DRONE_COLORS)]
        drone = drones[did-1] if did <= len(drones) else None

        # Status dot
        if ds['phase'] == 'COMPLETE':
            dot = (60, 255, 130)
        elif ds['phase'] == 'SCANNING':
            dot = SCAN_ACTIVE
        else:
            dot = DIM
        pygame.draw.circle(surf, dot, (WIDTH-rw+8, y+6), 4)
        surf.blit(font_s.render(f"DRONE {did} — Sector {ds['sector']+1}", True, dc),
                  (WIDTH-rw+16, y)); y += 14

        # Progress bar
        bw = rw - 40
        pygame.draw.rect(surf, (30,35,50), (WIDTH-rw+16, y, bw, 8), border_radius=3)
        fill = int(bw * ds['progress'] / 100)
        if fill > 0:
            pygame.draw.rect(surf, dc, (WIDTH-rw+16, y, fill, 8), border_radius=3)
        surf.blit(font_s.render(f"{ds['progress']:.0f}%", True, TEXT), (WIDTH-40, y-2)); y += 14

        surf.blit(font_s.render(f"📸 {ds['photos']} photos | {ds['remaining']} remaining",
                               True, DIM), (WIDTH-rw+16, y)); y += 16
        surf.blit(font_s.render(f"Phase: {ds['phase']}", True, (80,95,115)),
                  (WIDTH-rw+16, y)); y += 20

    # Total photos
    total = status['total_photos']
    total_pts = status['total_points']
    surf.blit(font_l.render(f"📸 Total: {total}/{total_pts} scan points captured", True, TEXT),
              (WIDTH-rw, y+5))

    # Bottom bar
    bb = pygame.Surface((WIDTH, 32), pygame.SRCALPHA); bb.fill(PANEL_BG)
    surf.blit(bb, (0, HEIGHT-32))
    surf.blit(font_s.render(
        "LEFT-CLICK: Place Structure  |  SPACE: Start Inspection  |  "
        "1-7: Drones  |  +/-: Orbit Radius  |  R: Reset  |  SCROLL: Zoom  |  ESC: Quit",
        True, DIM), (15, HEIGHT-26))

# ==================== MAIN ====================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("🏗️ Structural Inspection — Autonomous Drone Scanner")
    clock = pygame.time.Clock()
    cam = Camera()

    try:
        font_s = pygame.font.SysFont("monospace", 13)
        font_l = pygame.font.SysFont("monospace", 15, bold=True)
    except:
        font_s = pygame.font.Font(None, 15)
        font_l = pygame.font.Font(None, 19)

    # Default structure
    structure = Structure(center_x=0, center_y=0, radius=15, height=50, name="Tower")
    config = InspectionConfig(orbit_radius=35, alt_top=50, alt_bottom=10, alt_bands=5,
                               orbit_points_per_sector=10)
    num_drones = 4
    planner = StructuralInspectionPlanner(structure, config, num_drones)

    # Create drones at random positions around structure
    drones = []
    for i in range(num_drones):
        a = random.uniform(0, 2*math.pi)
        r = random.uniform(50, 80)
        d = SimDrone(i+1, DRONE_COLORS[i % len(DRONE_COLORS)],
                     structure.center_x + r*math.cos(a),
                     structure.center_y + r*math.sin(a))
        drones.append(d)

    mission_started = False
    t0 = time.time()
    running = True

    while running:
        dt = 1.0 / FPS
        t = time.time()

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: running = False
                if ev.key == pygame.K_SPACE and not mission_started:
                    planner.start_inspection()
                    mission_started = True
                    t0 = time.time()
                    for d in drones:
                        d.active = True
                        d.status = "Positioning"
                if ev.key == pygame.K_r:
                    # Reset
                    mission_started = False
                    planner = StructuralInspectionPlanner(structure, config, num_drones)
                    drones.clear()
                    for i in range(num_drones):
                        a = random.uniform(0, 2*math.pi)
                        r = random.uniform(50, 80)
                        d = SimDrone(i+1, DRONE_COLORS[i % len(DRONE_COLORS)],
                                     structure.center_x + r*math.cos(a),
                                     structure.center_y + r*math.sin(a))
                        drones.append(d)
                # Drone count
                for n in range(1, 8):
                    if ev.key == getattr(pygame, f'K_{n}', None):
                        num_drones = n
                        mission_started = False
                        planner = StructuralInspectionPlanner(structure, config, num_drones)
                        drones.clear()
                        for i in range(num_drones):
                            a = random.uniform(0, 2*math.pi)
                            r = random.uniform(50, 80)
                            d = SimDrone(i+1, DRONE_COLORS[i % len(DRONE_COLORS)],
                                         structure.center_x + r*math.cos(a),
                                         structure.center_y + r*math.sin(a))
                            drones.append(d)
                if ev.key == pygame.K_EQUALS or ev.key == pygame.K_PLUS:
                    config.orbit_radius += 5
                    if not mission_started:
                        planner = StructuralInspectionPlanner(structure, config, num_drones)
                if ev.key == pygame.K_MINUS:
                    config.orbit_radius = max(structure.radius + 5, config.orbit_radius - 5)
                    if not mission_started:
                        planner = StructuralInspectionPlanner(structure, config, num_drones)

            if ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1 and ev.pos[1] > 55 and ev.pos[1] < HEIGHT-35:
                    if ev.pos[0] < WIDTH - 300:
                        mx, my = cam.s2w(*ev.pos)
                        structure.center_x = mx
                        structure.center_y = my
                        if not mission_started:
                            planner = StructuralInspectionPlanner(structure, config, num_drones)
            if ev.type == pygame.MOUSEWHEEL:
                cam.zoom(ev.y)

        # Move drones toward their scan targets
        if mission_started:
            for d in drones:
                target = planner.get_drone_target(d.id)
                if target:
                    d.target_x, d.target_y, d.target_alt, th = target
                    d.heading = th
                    arrived = d.move_toward_target(dt)
                    if arrived:
                        took_photo = planner.report_arrived(d.id, d.x, d.y)
                        if took_photo:
                            d.photos += 1
                            d.status = f"Scanning ({d.photos} 📸)"
                else:
                    d.status = "Complete ✓"
                    d.active = False
        else:
            # Hover in place
            for d in drones:
                d.alt = max(d.alt, 0) + (20 - d.alt) * 0.02

        # Camera follow
        if drones:
            active_d = [d for d in drones if d.active]
            if active_d:
                cam.x = sum(d.x for d in active_d) / len(active_d)
                cam.y = sum(d.y for d in active_d) / len(active_d)
            else:
                cam.x = structure.center_x
                cam.y = structure.center_y

        # Draw
        screen.fill(BG)
        draw_grid(screen, cam)
        draw_orbit_path(screen, cam, structure, config)
        draw_sectors(screen, cam, planner)
        draw_scan_points(screen, cam, planner, font_s)
        draw_structure(screen, cam, structure)
        for d in drones:
            draw_drone(screen, cam, d, font_s, t)
        draw_hud(screen, planner, drones, t - t0 if mission_started else 0, font_s, font_l, num_drones)

        # Cursor preview
        if pygame.mouse.get_focused():
            mp = pygame.mouse.get_pos()
            if 55 < mp[1] < HEIGHT-35 and mp[0] < WIDTH-300:
                mx, my = cam.s2w(*mp)
                d = math.sqrt((mx-structure.center_x)**2 + (my-structure.center_y)**2)
                screen.blit(font_s.render(f"{d:.0f}m from center", True, DIM), (mp[0]+12, mp[1]-10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()
