#!/usr/bin/env python3
"""
INTERACTIVE SWARM COMMANDER v3
================================
DJI-style touchscreen waypoint control for ArduPilot SITL swarm.
Left-click to set waypoints. Right-click for obstacles. Full APF avoidance.
"""

import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

import subprocess, os, sys, time, math, signal, shutil, threading, random
import pygame
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ==================== CONFIG ====================
NUM_DRONES = 5
ARDUCOPTER_BIN = "/home/q/ardupilot/build/sitl/bin/arducopter"
HOME_LAT, HOME_LON = -35.363261, 149.165230
HOME_ALT = 584
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SPACING = 18.0
AVOID_RADIUS = 14.0
OBSTACLE_REPULSE = 30.0
CRUISE_ALT = 20.0
EARTH_R = 6378137.0

# Pygame
WIDTH, HEIGHT = 1400, 900
FPS = 30
MIN_ZOOM, MAX_ZOOM = 0.08, 2.0

# ==================== THEME ====================
class Theme:
    bg = (8, 10, 18)
    grid = (18, 22, 32)
    grid_major = (28, 35, 50)
    grid_axis = (45, 55, 75)
    panel_bg = (12, 14, 22, 220)
    text = (200, 210, 225)
    text_dim = (100, 115, 135)
    text_bright = (240, 245, 255)
    accent = (0, 180, 255)
    danger = (255, 60, 70)
    warning = (255, 200, 50)
    success = (60, 255, 130)
    waypoint = (255, 80, 120)
    waypoint_line = (255, 80, 120, 80)
    obstacle = (255, 50, 50)
    obstacle_zone = (255, 40, 40, 25)
    home = (255, 230, 80)
    formation_line = (35, 45, 65)
    proximity_warn = (255, 180, 50)
    proximity_danger = (255, 60, 60)
    drone_colors = [
        (0, 200, 255),
        (255, 130, 40),
        (80, 255, 130),
        (255, 220, 50),
        (190, 100, 255),
        (255, 80, 170),
        (100, 230, 210),
    ]

T = Theme()

# ==================== FORMATIONS ====================
FORMATIONS = {
    'V':      {'slots': [(0,0),(-1,-1),(-1,1),(-2,-2),(-2,2),(-3,-3),(-3,3)], 'icon': '◁'},
    'ARROW':  {'slots': [(0,0),(-1,-1),(-1,1),(-2,0),(-3,-1),(-3,1),(-4,0)], 'icon': '⬆'},
    'CIRCLE': {'slots': [(0,0)]+[(math.cos(i*math.pi/3),math.sin(i*math.pi/3)) for i in range(6)], 'icon': '◯'},
    'WALL':   {'slots': [(0,-3),(0,-2),(0,-1),(0,0),(0,1),(0,2),(0,3)], 'icon': '▬'},
    'LINE':   {'slots': [(0,0),(-1,0),(-2,0),(-3,0),(-4,0),(-5,0),(-6,0)], 'icon': '│'},
}
FORMATION_KEYS = list(FORMATIONS.keys())

# ==================== GPS MATH ====================
def gps_to_m(lat, lon):
    dx = (lon - HOME_LON) * EARTH_R * math.cos(math.radians(HOME_LAT)) * math.pi / 180.0
    dy = (lat - HOME_LAT) * EARTH_R * math.pi / 180.0
    return dx, dy

def m_to_gps(mx, my):
    lat = HOME_LAT + (my / EARTH_R) * (180.0 / math.pi)
    lon = HOME_LON + (mx / (EARTH_R * math.cos(math.pi * HOME_LAT / 180.0))) * (180.0 / math.pi)
    return lat, lon

def offset_gps(blat, blon, dn, de):
    return m_to_gps(*[a+b for a,b in zip(gps_to_m(blat, blon), [de, dn])])

def dist2d(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

# ==================== CAMERA ====================
class Camera:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.mpp = 0.4  # meters per pixel
        self.target_x = 0.0
        self.target_y = 0.0
        self.follow = True  # auto-follow swarm

    def world_to_screen(self, mx, my):
        px = WIDTH // 2 + (mx - self.x) / self.mpp
        py = HEIGHT // 2 - (my - self.y) / self.mpp
        return int(px), int(py)

    def screen_to_world(self, sx, sy):
        mx = (sx - WIDTH // 2) * self.mpp + self.x
        my = -(sy - HEIGHT // 2) * self.mpp + self.y
        return mx, my

    def zoom(self, delta):
        self.mpp = max(MIN_ZOOM, min(MAX_ZOOM, self.mpp - delta * 0.04))

    def update(self, drones):
        if self.follow:
            active = [d for d in drones if d.alt > 1]
            if active:
                self.target_x = sum(gps_to_m(d.lat, d.lon)[0] for d in active) / len(active)
                self.target_y = sum(gps_to_m(d.lat, d.lon)[1] for d in active) / len(active)
        # Smooth lerp
        self.x += (self.target_x - self.x) * 0.08
        self.y += (self.target_y - self.y) * 0.08

# ==================== SITL ====================
sitl_procs = []

def launch_sitl(n):
    offsets = []
    for i in range(n):
        d = os.path.join(BASE_DIR, f"sitl_v3_{i+1}")
        if os.path.exists(d): shutil.rmtree(d)
        os.makedirs(d)
        if i == 0:
            ox, oy = 0, 0
        else:
            a = random.uniform(0, 2*math.pi)
            r = random.uniform(25, 60)
            ox, oy = r*math.cos(a), r*math.sin(a)
        offsets.append((ox, oy))
        slat, slon = m_to_gps(ox, oy)
        pf = os.path.join(d, "default.parm")
        with open(pf, 'w') as f:
            f.write(f"SYSID_THISMAV {i+1}\nFRAME_CLASS 1\nFRAME_TYPE 1\n"
                    f"ARMING_CHECK 0\nSIM_SPEEDUP 1\nBRD_SAFETYENABLE 0\n"
                    f"BATT_MONITOR 0\nDISARM_DELAY 0\n")
        p = subprocess.Popen(
            [ARDUCOPTER_BIN, f"-I{i}", "--model", "quad",
             "--home", f"{slat},{slon},{HOME_ALT},{random.randint(0,359)}",
             "--defaults", pf],
            cwd=d, stdout=open(os.path.join(d,"sitl.log"),'w'), stderr=subprocess.STDOUT)
        sitl_procs.append(p)
    return offsets

def cleanup(*a):
    for p in sitl_procs:
        try: p.kill()
        except: pass
    subprocess.run(["pkill","-9","-f","arducopter"], capture_output=True)
    pygame.quit(); sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ==================== DRONE STATE ====================
class Drone:
    def __init__(self, did, color, smx=0, smy=0):
        self.id = did
        self.color = color
        self.vehicle = None
        self.lat, self.lon, self.alt = HOME_LAT, HOME_LON, 0.0
        self.heading = 0.0
        self.armed = False
        self.mode = "INIT"
        self.trail = []
        self.target_lat, self.target_lon = HOME_LAT, HOME_LON
        self.status = "Spawning..."
        self.spawn_mx, self.spawn_my = smx, smy
        self.is_leader = (did == 1)
        self.avoid_active = False
        self.speed = 0.0
        self.prev_pos = None

    def update(self):
        if not self.vehicle: return
        try:
            loc = self.vehicle.location.global_relative_frame
            if loc and loc.lat != 0:
                old_mx, old_my = gps_to_m(self.lat, self.lon)
                self.lat, self.lon = loc.lat, loc.lon
                self.alt = loc.alt or 0
                mx, my = gps_to_m(self.lat, self.lon)
                self.trail.append((mx, my, time.time()))
                if len(self.trail) > 600: self.trail.pop(0)
                self.speed = dist2d(old_mx, old_my, mx, my) * FPS
            self.armed = self.vehicle.armed
            self.mode = self.vehicle.mode.name
            h = self.vehicle.heading
            if h is not None: self.heading = math.radians(h)
        except: pass

# ==================== AVOIDANCE (APF) ====================
def apf_goto(drone, tlat, tlon, all_drones, obstacles):
    if not drone.vehicle or not drone.armed: return
    cmx, cmy = gps_to_m(drone.lat, drone.lon)
    tmx, tmy = gps_to_m(tlat, tlon)

    # Attractive
    dt = dist2d(cmx, cmy, tmx, tmy)
    if dt > 0.5:
        ax = (tmx - cmx) / dt * min(dt, 10)
        ay = (tmy - cmy) / dt * min(dt, 10)
    else:
        ax, ay = 0, 0

    # Repulsive (drones)
    rx, ry = 0.0, 0.0
    drone.avoid_active = False
    for o in all_drones:
        if o.id == drone.id or o.alt < 2: continue
        ox, oy = gps_to_m(o.lat, o.lon)
        d = dist2d(cmx, cmy, ox, oy)
        if d < AVOID_RADIUS and d > 0.1:
            s = (1/d - 1/AVOID_RADIUS) * 10
            rx += (cmx - ox) / d * s
            ry += (cmy - oy) / d * s
            drone.avoid_active = True

    # Repulsive (obstacles)
    for obx, oby, obr in obstacles:
        d = dist2d(cmx, cmy, obx, oby) - obr
        if d < OBSTACLE_REPULSE and d > 0.1:
            s = (1/max(d,0.5) - 1/OBSTACLE_REPULSE) * 20
            rx += (cmx - obx) / (d+obr) * s
            ry += (cmy - oby) / (d+obr) * s
            drone.avoid_active = True

    fx = cmx + ax + rx
    fy = cmy + ay + ry
    flat, flon = m_to_gps(fx, fy)
    drone.target_lat, drone.target_lon = flat, flon
    drone.vehicle.simple_goto(LocationGlobalRelative(flat, flon, CRUISE_ALT))

# ==================== SWARM COMMANDER ====================
class SwarmCommander:
    def __init__(self, drones):
        self.drones = drones
        self.waypoints = []       # List of (mx, my)
        self.current_wp = None    # Current active waypoint (mx, my)
        self.formation = 'V'
        self.heading = 0.0        # Auto-calculated from movement
        self.obstacles = []       # (mx, my, radius)
        self.phase = "LAUNCHING"
        self.auto_heading = True
        self.prev_target = None
        self.wp_reached_dist = 8.0  # meters to consider waypoint reached
        self.ready = False

    def add_waypoint(self, mx, my):
        self.waypoints.append((mx, my))
        if self.current_wp is None:
            self._next_waypoint()

    def _next_waypoint(self):
        if self.waypoints:
            self.current_wp = self.waypoints.pop(0)
            # Auto heading from prev to current
            if self.prev_target and self.auto_heading:
                dx = self.current_wp[0] - self.prev_target[0]
                dy = self.current_wp[1] - self.prev_target[1]
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    self.heading = math.atan2(dx, dy)  # heading is clockwise from north
        else:
            self.current_wp = None

    def clear_waypoints(self):
        self.waypoints.clear()
        self.current_wp = None

    def add_obstacle(self, mx, my, r=6.0):
        self.obstacles.append((mx, my, r))

    def clear_obstacles(self):
        self.obstacles.clear()

    def set_formation(self, name):
        if name in FORMATIONS:
            self.formation = name

    def update(self):
        if not self.ready: return
        if self.current_wp is None: return

        # Check if leader reached waypoint
        leader = self.drones[0]
        if leader.alt > 2:
            lmx, lmy = gps_to_m(leader.lat, leader.lon)
            d = dist2d(lmx, lmy, self.current_wp[0], self.current_wp[1])
            if d < self.wp_reached_dist:
                self.prev_target = self.current_wp
                self._next_waypoint()
                return

        # Command formation
        bp = FORMATIONS[self.formation]['slots']
        tlat, tlon = m_to_gps(self.current_wp[0], self.current_wp[1])

        for drone in self.drones:
            idx = drone.id - 1
            if idx >= len(bp) or not drone.vehicle or not drone.armed:
                continue
            sfwd, sright = bp[idx]
            rn = (sfwd * math.cos(self.heading) - sright * math.sin(self.heading)) * SPACING
            re = (sfwd * math.sin(self.heading) + sright * math.cos(self.heading)) * SPACING
            slot_lat, slot_lon = offset_gps(tlat, tlon, rn, re)
            apf_goto(drone, slot_lat, slot_lon, self.drones, self.obstacles)
            drone.status = f"{self.formation}[{drone.id}]"

# ==================== DRAWING ====================
def draw_grid(surf, cam):
    step = 10 if cam.mpp < 0.6 else 20 if cam.mpp < 1.0 else 50
    major = step * 5
    for gx in range(-500, 501, step):
        sx, _ = cam.world_to_screen(gx, 0)
        if 0 <= sx <= WIDTH:
            c = T.grid_axis if gx == 0 else T.grid_major if gx % major == 0 else T.grid
            pygame.draw.line(surf, c, (sx, 0), (sx, HEIGHT), 1)
    for gy in range(-500, 501, step):
        _, sy = cam.world_to_screen(0, gy)
        if 0 <= sy <= HEIGHT:
            c = T.grid_axis if gy == 0 else T.grid_major if gy % major == 0 else T.grid
            pygame.draw.line(surf, c, (0, sy), (WIDTH, sy), 1)

def draw_home(surf, cam, font):
    sx, sy = cam.world_to_screen(0, 0)
    for r in [14, 8]:
        pygame.draw.circle(surf, T.home, (sx, sy), r, 1)
    pygame.draw.line(surf, T.home, (sx-18, sy), (sx+18, sy), 1)
    pygame.draw.line(surf, T.home, (sx, sy-18), (sx, sy+18), 1)
    surf.blit(font.render("HOME", True, T.home), (sx+16, sy-6))

def draw_waypoints(surf, cam, commander, font):
    # Draw waypoint queue path
    points = []
    if commander.current_wp:
        points.append(commander.current_wp)
    points.extend(commander.waypoints)

    if len(points) > 1:
        screen_pts = [cam.world_to_screen(p[0], p[1]) for p in points]
        for i in range(len(screen_pts)-1):
            # Dashed line
            p1, p2 = screen_pts[i], screen_pts[i+1]
            dx, dy = p2[0]-p1[0], p2[1]-p1[1]
            l = math.sqrt(dx*dx+dy*dy)
            if l > 0:
                for t in range(0, int(l), 12):
                    s = t/l
                    e = min((t+6)/l, 1.0)
                    sp = (int(p1[0]+dx*s), int(p1[1]+dy*s))
                    ep = (int(p1[0]+dx*e), int(p1[1]+dy*e))
                    pygame.draw.line(surf, T.waypoint, sp, ep, 2)

    # Draw waypoint markers
    for i, (wx, wy) in enumerate(points):
        sx, sy = cam.world_to_screen(wx, wy)
        is_current = (i == 0 and commander.current_wp)
        if is_current:
            pulse = abs(math.sin(time.time() * 3)) * 8 + 14
            pygame.draw.circle(surf, T.waypoint, (sx, sy), int(pulse), 2)
            pygame.draw.circle(surf, T.waypoint, (sx, sy), int(pulse+12), 1)
            pygame.draw.line(surf, T.waypoint, (sx-10, sy), (sx+10, sy), 1)
            pygame.draw.line(surf, T.waypoint, (sx, sy-10), (sx, sy+10), 1)
            surf.blit(font.render("TARGET", True, T.waypoint), (sx+16, sy-8))
        else:
            num = i if commander.current_wp else i + 1
            pygame.draw.circle(surf, T.waypoint, (sx, sy), 8, 2)
            lbl = font.render(str(num), True, T.waypoint)
            surf.blit(lbl, (sx - lbl.get_width()//2, sy - lbl.get_height()//2))

def draw_obstacles(surf, cam):
    for ox, oy, r in commander_ref.obstacles:
        sx, sy = cam.world_to_screen(ox, oy)
        pr = int(r / cam.mpp)
        zr = int(OBSTACLE_REPULSE / cam.mpp)
        # Danger zone
        zs = pygame.Surface((zr*2, zr*2), pygame.SRCALPHA)
        pygame.draw.circle(zs, T.obstacle_zone, (zr, zr), zr)
        surf.blit(zs, (sx-zr, sy-zr))
        # Body
        pygame.draw.circle(surf, T.obstacle, (sx, sy), max(pr, 3))
        pygame.draw.circle(surf, (255, 150, 150), (sx, sy), max(pr, 3), 1)

def draw_spawns(surf, cam, drones, font):
    for d in drones:
        sx, sy = cam.world_to_screen(d.spawn_mx, d.spawn_my)
        pygame.draw.circle(surf, T.text_dim, (sx, sy), 5, 1)
        surf.blit(font.render(f"S{d.id}", True, T.text_dim), (sx+7, sy-5))

def draw_drone(surf, drone, cam, font, t):
    mx, my = gps_to_m(drone.lat, drone.lon)
    sx, sy = cam.world_to_screen(mx, my)

    # Trail
    if len(drone.trail) > 2:
        ts = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        for i in range(1, len(drone.trail)):
            age = t - drone.trail[i][2]
            if age > 30: continue
            alpha = int(max(10, 100 - age * 3))
            w = max(1, int(3 - age * 0.08))
            p1 = cam.world_to_screen(drone.trail[i-1][0], drone.trail[i-1][1])
            p2 = cam.world_to_screen(drone.trail[i][0], drone.trail[i][1])
            pygame.draw.line(ts, (*drone.color, alpha), p1, p2, w)
        surf.blit(ts, (0, 0))

    # Target line
    tmx, tmy = gps_to_m(drone.target_lat, drone.target_lon)
    tsx, tsy = cam.world_to_screen(tmx, tmy)
    pygame.draw.line(surf, (*drone.color[:3], 50) if len(drone.color)>=3 else drone.color, (sx,sy), (tsx,tsy), 1)

    # Avoidance indicator ring
    if drone.avoid_active:
        ar = int(AVOID_RADIUS / cam.mpp)
        avs = pygame.Surface((ar*2, ar*2), pygame.SRCALPHA)
        pygame.draw.circle(avs, (255, 80, 80, 35), (ar, ar), ar, 2)
        surf.blit(avs, (sx-ar, sy-ar))

    # Body
    size = max(8, int(16 / max(cam.mpp, 0.3)))
    h = drone.heading

    # Leader glow
    if drone.is_leader:
        gs = pygame.Surface((80, 80), pygame.SRCALPHA)
        pygame.draw.circle(gs, (*drone.color, 25), (40, 40), 30)
        surf.blit(gs, (sx-40, sy-40))

    # Arms + spinning props
    prop_t = t * 12 + drone.id * 1.5
    for ao in [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]:
        ex = sx + int(size * math.cos(h + ao))
        ey = sy - int(size * math.sin(h + ao))
        pygame.draw.line(surf, drone.color, (sx, sy), (ex, ey), 2)
        for blade in range(3):
            bx = ex + int(5 * math.cos(prop_t + blade * 2.094 + ao))
            by = ey - int(5 * math.sin(prop_t + blade * 2.094 + ao))
            pygame.draw.line(surf, drone.color, (ex, ey), (bx, by), 1)

    # Center
    pygame.draw.circle(surf, T.text_bright, (sx, sy), 3 if drone.is_leader else 2)

    # Nose
    nx = sx + int(size * 1.3 * math.cos(h + math.pi/2))
    ny = sy - int(size * 1.3 * math.sin(h + math.pi/2))
    pygame.draw.line(surf, T.danger if drone.is_leader else T.warning, (sx, sy), (nx, ny), 2)

    # Labels
    tag = "★ LEADER" if drone.is_leader else f"D{drone.id}"
    surf.blit(font.render(tag, True, T.text_bright if drone.is_leader else drone.color), (sx+20, sy-12))
    surf.blit(font.render(f"{drone.alt:.0f}m {drone.speed:.1f}m/s", True, T.text_dim), (sx+20, sy+2))

def draw_formation_lines(surf, drones, cam):
    airborne = [d for d in drones if d.alt > 5]
    for i in range(len(airborne)):
        for j in range(i+1, len(airborne)):
            p1 = cam.world_to_screen(*gps_to_m(airborne[i].lat, airborne[i].lon))
            p2 = cam.world_to_screen(*gps_to_m(airborne[j].lat, airborne[j].lon))
            d = dist2d(*gps_to_m(airborne[i].lat, airborne[i].lon),
                       *gps_to_m(airborne[j].lat, airborne[j].lon))
            if d < AVOID_RADIUS: lc = T.proximity_danger
            elif d < AVOID_RADIUS * 2: lc = T.proximity_warn
            else: lc = T.formation_line
            pygame.draw.line(surf, lc, p1, p2, 1)

def draw_hud(surf, drones, commander, elapsed, font_s, font_l):
    # Top bar
    tb = pygame.Surface((WIDTH, 50), pygame.SRCALPHA); tb.fill(T.panel_bg)
    surf.blit(tb, (0, 0))
    surf.blit(font_l.render("SWARM COMMANDER • ArduPilot SITL Live", True, T.text_bright), (20, 5))
    surf.blit(font_l.render(f"▸ {commander.phase}", True, T.waypoint), (20, 26))

    # Formation selector (top right)
    fx = WIDTH - 400
    surf.blit(font_s.render("FORMATION:", True, T.text_dim), (fx, 8))
    for i, name in enumerate(FORMATION_KEYS):
        bx = fx + 100 + i * 58
        active = (name == commander.formation)
        bc = T.accent if active else T.text_dim
        pygame.draw.rect(surf, bc, (bx, 5, 52, 20), 0 if active else 1, border_radius=4)
        tc = T.bg if active else T.text_dim
        surf.blit(font_s.render(name, True, tc), (bx + 4, 8))
    surf.blit(font_s.render(f"T+{int(elapsed)}s  WP:{len(commander.waypoints)} queued", True, T.text_dim),
              (fx + 100, 30))

    # Right panel — telemetry
    pw = 300
    ph = 25 + len(drones) * 44
    panel = pygame.Surface((pw, ph), pygame.SRCALPHA); panel.fill(T.panel_bg)
    surf.blit(panel, (WIDTH-pw-10, 60))
    y = 68
    surf.blit(font_s.render("━━ FLEET STATUS ━━", True, T.text_dim), (WIDTH-pw, y)); y += 18
    for d in drones:
        dot = T.success if d.armed else T.danger if d.vehicle else T.text_dim
        pygame.draw.circle(surf, dot, (WIDTH-pw+8, y+6), 4)
        tag = "★ LDR" if d.is_leader else f"  D{d.id}"
        surf.blit(font_s.render(f"{tag} [{d.mode}]", True, d.color), (WIDTH-pw+16, y)); y += 14
        mx, my = gps_to_m(d.lat, d.lon)
        surf.blit(font_s.render(f"   {d.alt:.0f}m ({mx:.0f},{my:.0f}) {d.speed:.1f}m/s",
                                True, T.text_dim), (WIDTH-pw+16, y)); y += 14
        surf.blit(font_s.render(f"   {d.status}", True, (80,95,115)), (WIDTH-pw+16, y)); y += 18

    # Bottom bar
    bb = pygame.Surface((WIDTH, 32), pygame.SRCALPHA); bb.fill(T.panel_bg)
    surf.blit(bb, (0, HEIGHT-32))
    surf.blit(font_s.render(
        "LEFT-CLICK: Set Waypoint  │  RIGHT-CLICK: Place Obstacle  │  "
        "C: Clear Waypoints  │  X: Clear Obstacles  │  1-5: Formation  │  "
        "SCROLL: Zoom  │  F: Toggle Follow  │  ESC: Quit",
        True, T.text_dim), (15, HEIGHT-26))

def draw_radar(surf, drones, commander, cam):
    rs = 150
    rx, ry = 12, HEIGHT - 195
    radar = pygame.Surface((rs, rs), pygame.SRCALPHA)
    radar.fill((0, 0, 0, 170))
    pygame.draw.rect(radar, T.text_dim, (0, 0, rs, rs), 1)
    cx, cy = rs//2, rs//2
    sc = 1.5
    pygame.draw.circle(radar, T.home, (cx, cy), 3, 1)
    for d in drones:
        mx, my = gps_to_m(d.lat, d.lon)
        rpx, rpy = cx+int(mx/sc), cy-int(my/sc)
        if 2<=rpx<rs-2 and 2<=rpy<rs-2:
            pygame.draw.circle(radar, d.color, (rpx, rpy), 3 if d.is_leader else 2)
    for ox, oy, r in commander.obstacles:
        rpx, rpy = cx+int(ox/sc), cy-int(oy/sc)
        if 2<=rpx<rs-2 and 2<=rpy<rs-2:
            pygame.draw.circle(radar, T.obstacle, (rpx, rpy), max(2,int(r/sc)), 1)
    if commander.current_wp:
        wpx, wpy = cx+int(commander.current_wp[0]/sc), cy-int(commander.current_wp[1]/sc)
        if 2<=wpx<rs-2 and 2<=wpy<rs-2:
            pygame.draw.circle(radar, T.waypoint, (wpx, wpy), 4, 1)
    font_r = pygame.font.SysFont("monospace", 10)
    radar.blit(font_r.render("RADAR", True, T.text_dim), (rs//2-16, 3))
    surf.blit(radar, (rx, ry))

# ==================== INIT THREAD ====================
commander_ref = None

def init_thread(drones, commander):
    commander.phase = "CONNECTING TO SITL"
    for d in drones:
        d.status = "Connecting..."
        port = 5760 + (d.id-1)*10
        for attempt in range(12):
            try:
                d.vehicle = connect(f"tcp:127.0.0.1:{port}", wait_ready=True, timeout=30)
                d.status = "Connected ✓"
                break
            except:
                d.status = f"Retry {attempt+1}..."
                time.sleep(3)

    commander.phase = "CALIBRATE & TAKEOFF"
    for d in drones:
        if not d.vehicle: continue
        d.status = "Calibrating..."
        msg = d.vehicle.message_factory.command_long_encode(
            0,0, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0,0,0,0,4,0,0)
        d.vehicle.send_mavlink(msg); d.vehicle.flush()
        time.sleep(15)
        d.status = "GUIDED..."
        d.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)
        d.status = "EKF wait..."
        for _ in range(120):
            try:
                if d.vehicle.ekf_ok: break
            except: pass
            time.sleep(1)
        d.status = "Arming..."
        d.vehicle.armed = True
        for _ in range(30):
            if d.vehicle.armed: break
            d.vehicle.armed = True
            time.sleep(1)
        if not d.vehicle.armed:
            d.status = "ARM FAILED"
            continue
        d.status = "Takeoff..."
        d.vehicle.simple_takeoff(CRUISE_ALT)
        for _ in range(60):
            a = d.vehicle.location.global_relative_frame.alt
            if a and a >= CRUISE_ALT*0.9: break
            time.sleep(1)
        d.status = "Ready ✓"

    commander.phase = "READY — Click to set waypoints!"
    commander.ready = True

    # Set initial waypoint at home so they gather
    commander.add_waypoint(0, 0)

# ==================== MAIN ====================
def main():
    global commander_ref
    subprocess.run(["pkill","-9","-f","arducopter"], capture_output=True)
    time.sleep(2)

    print("🚀 Launching SITL fleet...")
    offsets = launch_sitl(NUM_DRONES)
    print("⏳ Waiting 15s for EKF...")
    time.sleep(15)

    drones = [Drone(i+1, T.drone_colors[i%len(T.drone_colors)], *offsets[i]) for i in range(NUM_DRONES)]
    commander = SwarmCommander(drones)
    commander_ref = commander

    threading.Thread(target=init_thread, args=(drones, commander), daemon=True).start()

    # Avoidance loop thread
    def avoidance_loop():
        while True:
            if commander.ready:
                commander.update()
            time.sleep(0.5)
    threading.Thread(target=avoidance_loop, daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("🚁 Interactive Swarm Commander — ArduPilot SITL")
    clock = pygame.time.Clock()
    cam = Camera()

    try:
        font_s = pygame.font.SysFont("monospace", 13)
        font_l = pygame.font.SysFont("monospace", 15, bold=True)
    except:
        font_s = pygame.font.Font(None, 15)
        font_l = pygame.font.Font(None, 19)

    t0 = time.time()
    running = True

    while running:
        t = time.time()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: running = False
                if ev.key == pygame.K_f: cam.follow = not cam.follow
                if ev.key == pygame.K_c: commander.clear_waypoints()
                if ev.key == pygame.K_x: commander.clear_obstacles()
                # Formation keys
                for i, name in enumerate(FORMATION_KEYS):
                    if ev.key == getattr(pygame, f'K_{i+1}', None):
                        commander.set_formation(name)
            if ev.type == pygame.MOUSEBUTTONDOWN:
                # Ignore clicks on HUD areas
                if ev.pos[1] > 55 and ev.pos[1] < HEIGHT - 35:
                    if ev.pos[0] < WIDTH - 310:
                        mx, my = cam.screen_to_world(*ev.pos)
                        if ev.button == 1:  # Left click = waypoint
                            commander.add_waypoint(mx, my)
                        elif ev.button == 3:  # Right click = obstacle
                            commander.add_obstacle(mx, my)
            if ev.type == pygame.MOUSEWHEEL:
                cam.zoom(ev.y)

        for d in drones: d.update()
        cam.update(drones)

        # Draw
        screen.fill(T.bg)
        draw_grid(screen, cam)
        draw_home(screen, cam, font_s)
        draw_spawns(screen, cam, drones, font_s)
        draw_obstacles(screen, cam)
        draw_waypoints(screen, cam, commander, font_s)
        draw_formation_lines(screen, drones, cam)
        for d in sorted(drones, key=lambda x: x.is_leader):
            draw_drone(screen, d, cam, font_s, t)
        draw_hud(screen, drones, commander, t - t0, font_s, font_l)
        draw_radar(screen, drones, commander, cam)

        # Mouse cursor waypoint preview
        if pygame.mouse.get_focused():
            mpos = pygame.mouse.get_pos()
            if 55 < mpos[1] < HEIGHT-35 and mpos[0] < WIDTH-310:
                mx, my = cam.screen_to_world(*mpos)
                # Show distance from leader
                if drones[0].alt > 2:
                    lmx, lmy = gps_to_m(drones[0].lat, drones[0].lon)
                    d = dist2d(lmx, lmy, mx, my)
                    screen.blit(font_s.render(f"{d:.0f}m", True, T.text_dim), (mpos[0]+15, mpos[1]-10))
                # Crosshair cursor
                pygame.draw.line(screen, (80,90,110), (mpos[0]-8, mpos[1]), (mpos[0]+8, mpos[1]), 1)
                pygame.draw.line(screen, (80,90,110), (mpos[0], mpos[1]-8), (mpos[0], mpos[1]+8), 1)

        pygame.display.flip()
        clock.tick(FPS)

    cleanup()

if __name__ == "__main__":
    main()
