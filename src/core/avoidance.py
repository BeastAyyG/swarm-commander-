"""
avoidance.py — Artificial Potential Field (APF) Collision Avoidance Engine
==========================================================================
Implements real-time repulsive field calculations for:
  - Inter-drone collision avoidance
  - Static obstacle avoidance
  - Boundary enforcement
"""

import math


class APFEngine:
    """
    Artificial Potential Field engine for multi-agent collision avoidance.
    
    The total force on each agent is:
        F_total = F_attractive(target) + Σ F_repulsive(neighbors) + Σ F_repulsive(obstacles)
    
    Args:
        drone_avoid_radius: Minimum inter-drone distance (meters)
        obstacle_avoid_radius: Obstacle repulsion range (meters)
        drone_repulse_gain: Strength of inter-drone repulsion
        obstacle_repulse_gain: Strength of obstacle repulsion
    """
    
    def __init__(self, drone_avoid_radius=14.0, obstacle_avoid_radius=30.0,
                 drone_repulse_gain=10.0, obstacle_repulse_gain=20.0):
        self.drone_avoid_radius = drone_avoid_radius
        self.obstacle_avoid_radius = obstacle_avoid_radius
        self.drone_repulse_gain = drone_repulse_gain
        self.obstacle_repulse_gain = obstacle_repulse_gain
    
    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def compute_attractive(self, cx, cy, tx, ty, max_force=10.0):
        """Compute attractive force toward target."""
        d = self._dist(cx, cy, tx, ty)
        if d < 0.5:
            return 0.0, 0.0
        fx = (tx - cx) / d * min(d, max_force)
        fy = (ty - cy) / d * min(d, max_force)
        return fx, fy
    
    def compute_drone_repulsion(self, cx, cy, neighbors):
        """
        Compute total repulsive force from neighboring drones.
        
        Args:
            cx, cy: Current drone position (meters)
            neighbors: List of (nx, ny) neighbor positions
        
        Returns:
            (fx, fy, is_active) — force vector and whether avoidance is engaged
        """
        fx, fy = 0.0, 0.0
        active = False
        
        for nx, ny in neighbors:
            d = self._dist(cx, cy, nx, ny)
            if d < self.drone_avoid_radius and d > 0.1:
                strength = (1.0 / d - 1.0 / self.drone_avoid_radius) * self.drone_repulse_gain
                fx += (cx - nx) / d * strength
                fy += (cy - ny) / d * strength
                active = True
        
        return fx, fy, active
    
    def compute_obstacle_repulsion(self, cx, cy, obstacles):
        """
        Compute total repulsive force from obstacles.
        
        Args:
            cx, cy: Current drone position (meters)
            obstacles: List of (ox, oy, radius) obstacles
        
        Returns:
            (fx, fy, is_active)
        """
        fx, fy = 0.0, 0.0
        active = False
        
        for ox, oy, radius in obstacles:
            d = self._dist(cx, cy, ox, oy) - radius
            if d < self.obstacle_avoid_radius and d > 0.1:
                strength = (1.0 / max(d, 0.5) - 1.0 / self.obstacle_avoid_radius) * self.obstacle_repulse_gain
                fx += (cx - ox) / (d + radius) * strength
                fy += (cy - oy) / (d + radius) * strength
                active = True
        
        return fx, fy, active
    
    def compute_total_force(self, cx, cy, tx, ty, neighbors, obstacles):
        """
        Compute the combined APF force vector.
        
        Returns:
            (final_x, final_y, avoid_active) — the target position with avoidance applied
        """
        # Attractive
        ax, ay = self.compute_attractive(cx, cy, tx, ty)
        
        # Repulsive (drones)
        drx, dry, d_active = self.compute_drone_repulsion(cx, cy, neighbors)
        
        # Repulsive (obstacles)
        orx, ory, o_active = self.compute_obstacle_repulsion(cx, cy, obstacles)
        
        # Sum forces
        final_x = cx + ax + drx + orx
        final_y = cy + ay + dry + ory
        
        return final_x, final_y, (d_active or o_active)
