#!/usr/bin/env python3
"""
Main application class for the touch screen interface
Particle blob app with four emitters controlled by Arduino sensors
"""

import pygame
import sys
import math
import random
import json
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Optional, Tuple
from datetime import datetime as dt
from functools import lru_cache

# Add parent directories to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.config import load_all_config
from hardware.arduino_interface import ArduinoInterface
from ui.renderer import Renderer, RenderLayer
from ui.tiles import TileManager
import pymunk
import numpy as np


# Particle configuration
PARTICLE_RADIUS = 1  # Smaller particles
PARTICLE_MASS = 0.05  # Lighter mass for smaller particles
PARTICLE_FRICTION = 0.2
PARTICLE_ELASTICITY = 0.0

# Physics
GRAVITY = (0, 1200)  # Gravity force (pixels/s^2)
DAMPING = 0.995

# Fluid forces
VISCOSITY = 0.10
PRESSURE = 1800.0
REST_DIST = PARTICLE_RADIUS * 2.2
NEIGHBOR_RADIUS = REST_DIST * 2.2

# Cohesion (surface tension)
COHESION = 260.0
COHESION_START = 0.9
COHESION_END = 2.0

# Spatial hash
CELL = int(NEIGHBOR_RADIUS)

# Rendering
RENDER_SCALE = 4
ISO_THRESHOLD = 1.15
FIELD_STRENGTH = 30.0  # Adjusted for smaller particles
FIELD_SOFTEN = 1.5  # Adjusted for smaller particles to maintain smooth blob appearance
METABALL_RENDER_MODE = "splat"  # "splat" (fast) or "field" (accurate)
SPLAT_MAX_RADIUS = 12

# Emission defaults (will be controlled by sensors)
BASE_PARTICLE_SPEED = 520
BASE_EMIT_INTERVAL = 0.8
EMISSION_SPREAD_DEG = 18

# Corridor scrolling effect - 3D perspective
CORRIDOR_SCROLL_SPEED = 200.0  # Units per second - how fast everything moves backward in Z
PARTICLE_LIFETIME = 60.0  # Seconds - greatly extended lifetime
HALLWAY_SEGMENT_LENGTH = 400  # Length of each hallway segment in Z-space
PERSPECTIVE_FOV = 800.0  # Field of view for perspective projection (larger = wider view)
VANISHING_POINT_Y = 0.5  # Y position of vanishing point (0.0 = top, 1.0 = bottom, 0.5 = center)
NEAR_PLANE_Z = 100.0  # Z distance of near plane (closest visible objects)
FAR_PLANE_Z = 5000.0  # Z distance of far plane (furthest visible objects)

# Forward launch configuration
FORWARD_LAUNCH_VELOCITY = -600.0  # Initial forward velocity (negative = toward camera, units per second)
FORWARD_LAUNCH_DECAY = 0.92  # Velocity decay per frame (0.92 = 8% reduction per frame)


def clamp(x, a, b):
    """Clamp value between a and b"""
    return a if x < a else (b if x > b else x)


def project_3d_to_2d(x, y, z, screen_width, screen_height):
    """
    Project 3D point (x, y, z) to 2D screen coordinates using perspective projection
    x, y: 2D position in world space
    z: Depth (distance from camera, increases as objects move backward)
    Returns: (screen_x, screen_y, scale_factor)
    """
    # Perspective projection: objects further away (larger z) appear smaller
    # Scale factor based on distance from camera
    scale = PERSPECTIVE_FOV / (PERSPECTIVE_FOV + z)
    
    # Vanishing point at center of screen (horizontally) and configurable Y position
    vanishing_x = screen_width / 2
    vanishing_y = screen_height * VANISHING_POINT_Y
    
    # Project to screen: move towards vanishing point based on depth
    screen_x = vanishing_x + (x - vanishing_x) * scale
    screen_y = vanishing_y + (y - vanishing_y) * scale
    
    return screen_x, screen_y, scale


def get_perspective_scale(z):
    """Get scale factor for object at depth z"""
    # Safety check: prevent division by zero
    denominator = PERSPECTIVE_FOV + z
    if abs(denominator) < 1e-6:  # Very small or zero
        return 1.0  # Default scale
    return PERSPECTIVE_FOV / denominator


def generate_decagon_vertices(center_x, center_y, radius):
    """
    Generate vertices for a regular decagon (10-sided polygon)
    Returns list of (x, y) tuples
    """
    vertices = []
    num_sides = 10
    for i in range(num_sides):
        angle = (2 * math.pi * i) / num_sides - math.pi / 2  # Start at top (-90 degrees)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    return vertices


def spatial_hash(positions):
    """Create spatial hash grid for neighbor finding"""
    grid = defaultdict(list)
    for i, (x, y) in enumerate(positions):
        cx = int(x // CELL)
        cy = int(y // CELL)
        grid[(cx, cy)].append(i)
    return grid


def neighbor_cells(cx, cy):
    """Get neighboring cells for spatial hash"""
    for ox in (-1, 0, 1):
        for oy in (-1, 0, 1):
            yield (cx + ox, cy + oy)


@lru_cache(maxsize=8)
def _get_metaball_grid(width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
    w2, h2 = width // RENDER_SCALE, height // RENDER_SCALE
    ys = np.arange(h2, dtype=np.float32)[:, None]
    xs = np.arange(w2, dtype=np.float32)[None, :]
    return xs, ys


def draw_metaballs(screen, particles, width, height, boundaries=None, margin=30, decagon_center_x=None, decagon_center_y=None, decagon_radius=None):
    """
    Draw particles using metaball rendering with flattening near decagon boundaries
    boundaries: List of boundary segments for flattening calculation
    decagon_center_x, decagon_center_y, decagon_radius: Decagon boundary parameters
    """
    if METABALL_RENDER_MODE == "splat":
        return draw_metaballs_splat(screen, particles, width, height)

    # Low-res buffer
    w2, h2 = width // RENDER_SCALE, height // RENDER_SCALE
    field = np.zeros((h2, w2), dtype=np.float32)

    # Precompute a grid of pixel coordinates (low-res)
    xs, ys = _get_metaball_grid(width, height)

    # Add each particle's contribution to the field
    for p in particles:
        # Get 2D position and Z-depth
        x, y = p["body"].position
        z_depth = p.get("z_depth", NEAR_PLANE_Z)
        
        # Project 3D to 2D using perspective
        proj_x, proj_y, scale = project_3d_to_2d(x, y, z_depth, width, height)
        
        # Scale particle size based on depth
        scaled_radius = PARTICLE_RADIUS * scale
        
        # Project to low-res buffer coordinates
        cx = (proj_x / RENDER_SCALE)
        cy = (proj_y / RENDER_SCALE)
        
        # Check if particle is flattened
        is_flattened = p.get("state") == "flattened"
        blocked_direction = p.get("blocked_direction")  # Normal vector from decagon surface
        
        # Calculate distance to decagon boundary and get surface normal
        flatten_strength = 0.0
        normal_x = 0.0
        normal_y = 0.0
        
        if decagon_center_x is not None and decagon_center_y is not None and decagon_radius is not None:
            # Project decagon center and radius to low-res coordinates
            decagon_cx = decagon_center_x / RENDER_SCALE
            decagon_cy = decagon_center_y / RENDER_SCALE
            decagon_r = decagon_radius / RENDER_SCALE
            
            # Calculate distance from particle to decagon center
            dx_to_center = cx - decagon_cx
            dy_to_center = cy - decagon_cy
            dist_to_center = math.sqrt(dx_to_center * dx_to_center + dy_to_center * dy_to_center)
            
            # Calculate distance to decagon edge
            dist_to_edge = dist_to_center - decagon_r
            
            # Flattening range (within 2x scaled particle radius)
            flatten_range = (scaled_radius * 2) / RENDER_SCALE
            
            if is_flattened and blocked_direction:
                # Particle is flattened - use stored normal vector
                flatten_strength = 1.0
                normal_x, normal_y = blocked_direction
            elif dist_to_edge < flatten_range and dist_to_edge >= -flatten_range:
                # Particle is near decagon boundary
                if dist_to_center > 1e-6:
                    # Calculate normal vector pointing outward from decagon center
                    normal_x = dx_to_center / dist_to_center
                    normal_y = dy_to_center / dist_to_center
                else:
                    normal_x = 1.0
                    normal_y = 0.0
                
                # Calculate flattening strength based on distance to edge
                if dist_to_edge >= 0:
                    # Outside decagon - strong flattening
                    t = min(dist_to_edge / flatten_range, 1.0) if flatten_range > 0 else 0.0
                    flatten_strength = 1.0 - t * 0.4  # 1.0 at edge, 0.6 at range
                else:
                    # Inside decagon - weaker flattening
                    t = min(-dist_to_edge / flatten_range, 1.0) if flatten_range > 0 else 0.0
                    flatten_strength = 0.6 * (1.0 - t)  # 0.6 at edge, 0.0 at range
                
                flatten_strength = clamp(flatten_strength, 0.0, 1.0)
        
        depth_field_strength = FIELD_STRENGTH * scale
        depth_field_soften = FIELD_SOFTEN * scale
        max_radius_sq = (depth_field_strength / ISO_THRESHOLD) - depth_field_soften
        if max_radius_sq <= 0:
            continue
        influence_radius = math.sqrt(max_radius_sq)
        if flatten_strength > 0.0:
            influence_radius *= 1.4

        x0 = max(int(cx - influence_radius), 0)
        x1 = min(int(cx + influence_radius) + 1, w2)
        y0 = max(int(cy - influence_radius), 0)
        y1 = min(int(cy + influence_radius) + 1, h2)
        if x1 <= x0 or y1 <= y0:
            continue

        xs_slice = xs[:, x0:x1]
        ys_slice = ys[y0:y1, :]

        # Apply flattening based on decagon surface normal
        if flatten_strength > 0.0 and (normal_x != 0.0 or normal_y != 0.0):
            # Flattening: make blob wider perpendicular to surface normal, shorter parallel to normal
            # The normal points outward, so we flatten along the normal direction
            flatten_mult = 1.2 if is_flattened else 0.8
            
            # Calculate tangent vector (perpendicular to normal)
            tangent_x = -normal_y
            tangent_y = normal_x
            
            # Transform coordinates: spread along tangent, squash along normal
            # Project (xs-cx, ys-cy) onto normal and tangent
            dx_local = xs_slice - cx
            dy_local = ys_slice - cy
            
            # Project onto normal and tangent using numpy operations
            dot_normal = dx_local * normal_x + dy_local * normal_y
            dot_tangent = dx_local * tangent_x + dy_local * tangent_y
            
            # Spread along tangent, squash along normal
            dx = dot_tangent / (1.0 + flatten_strength * flatten_mult * 0.5)  # Spread perpendicular
            dy = dot_normal * (1.0 + flatten_strength * flatten_mult)  # Squash parallel
            
            field[y0:y1, x0:x1] += depth_field_strength / (dx*dx + dy*dy + depth_field_soften)
        else:
            # Normal circular field for particles far from boundaries
            # Scale field strength by depth (farther particles contribute less)
            dx = xs_slice - cx
            dy = ys_slice - cy
            field[y0:y1, x0:x1] += depth_field_strength / (dx*dx + dy*dy + depth_field_soften)

    # Threshold to get silhouette
    mask = field >= ISO_THRESHOLD

    # Create an RGB image (low-res), then scale up
    img = np.zeros((h2, w2, 3), dtype=np.uint8)
    img[mask] = (120, 190, 255)  # Liquid color

    surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
    surf = pygame.transform.smoothscale(surf, (width, height))
    # Set black as the colorkey to make background transparent
    surf.set_colorkey((0, 0, 0))
    screen.blit(surf, (0, 0))


@lru_cache(maxsize=64)
def _get_splat_kernel(radius: int) -> np.ndarray:
    size = radius * 2 + 1
    ys = np.arange(size, dtype=np.float32)[:, None] - radius
    xs = np.arange(size, dtype=np.float32)[None, :] - radius
    kernel = FIELD_STRENGTH / (xs * xs + ys * ys + FIELD_SOFTEN)
    return kernel.astype(np.float32)


def draw_metaballs_splat(screen, particles, width, height):
    """Fast metaball rendering using cached kernels and local splats."""
    w2, h2 = width // RENDER_SCALE, height // RENDER_SCALE
    field = np.zeros((h2, w2), dtype=np.float32)

    for p in particles:
        x, y = p["body"].position
        z_depth = p.get("z_depth", NEAR_PLANE_Z)

        proj_x, proj_y, scale = project_3d_to_2d(x, y, z_depth, width, height)
        cx = proj_x / RENDER_SCALE
        cy = proj_y / RENDER_SCALE

        depth_field_strength = FIELD_STRENGTH * scale
        depth_field_soften = FIELD_SOFTEN * scale
        max_radius_sq = (depth_field_strength / ISO_THRESHOLD) - depth_field_soften
        if max_radius_sq <= 0:
            continue
        influence_radius = math.sqrt(max_radius_sq)
        radius = max(2, min(SPLAT_MAX_RADIUS, int(influence_radius)))
        kernel = _get_splat_kernel(radius) * scale

        x0 = int(cx) - radius
        y0 = int(cy) - radius
        x1 = x0 + kernel.shape[1]
        y1 = y0 + kernel.shape[0]

        if x1 <= 0 or y1 <= 0 or x0 >= w2 or y0 >= h2:
            continue

        sx0 = max(0, x0)
        sy0 = max(0, y0)
        sx1 = min(w2, x1)
        sy1 = min(h2, y1)

        kx0 = sx0 - x0
        ky0 = sy0 - y0
        kx1 = kx0 + (sx1 - sx0)
        ky1 = ky0 + (sy1 - sy0)

        field[sy0:sy1, sx0:sx1] += kernel[ky0:ky1, kx0:kx1]

    mask = field >= ISO_THRESHOLD
    img = np.zeros((h2, w2, 3), dtype=np.uint8)
    img[mask] = (120, 190, 255)
    surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
    surf = pygame.transform.smoothscale(surf, (width, height))
    surf.set_colorkey((0, 0, 0))
    screen.blit(surf, (0, 0))


class Emitter:
    """Emitter that emits particles based on sensor input"""
    def __init__(self, name, position_func, direction, spread_deg=EMISSION_SPREAD_DEG):
        self.name = name
        self.position_func = position_func
        dx, dy = direction
        mag = math.hypot(dx, dy) or 1.0
        self.dir = (dx / mag, dy / mag)
        self.spread = math.radians(spread_deg)
        
        # Sensor-controlled properties
        self.target_speed = BASE_PARTICLE_SPEED
        self.current_speed = BASE_PARTICLE_SPEED
        self.target_emit_interval = BASE_EMIT_INTERVAL
        self.current_emit_interval = BASE_EMIT_INTERVAL
        
        # Timing
        self.time_until_next_emit = BASE_EMIT_INTERVAL
        self.sensor_value = 0.0  # 0.0 to 1.0
        
    def update(self, dt, sensor_value=0.0):
        """
        Update emitter timing and properties based on sensor input
        sensor_value: 0.0 to 1.0, controls emission rate and velocity
        """
        # Store previous sensor value to detect changes
        prev_sensor = self.sensor_value
        self.sensor_value = sensor_value
        
        # Higher sensor value = faster emission and higher velocity
        # Map sensor value (0-1) to emission interval (BASE_EMIT_INTERVAL * 0.1 to BASE_EMIT_INTERVAL * 2.0)
        # Very fast emission when sensor value is high
        min_interval = BASE_EMIT_INTERVAL * 0.1  # Much faster minimum
        max_interval = BASE_EMIT_INTERVAL * 2.0
        self.target_emit_interval = max_interval - sensor_value * (max_interval - min_interval)
        
        # Map sensor value to speed (BASE_PARTICLE_SPEED * 0.5 to BASE_PARTICLE_SPEED * 2.0)
        min_speed = BASE_PARTICLE_SPEED * 0.5
        max_speed = BASE_PARTICLE_SPEED * 2.0
        self.target_speed = min_speed + sensor_value * (max_speed - min_speed)
        
        # Faster interpolation when sensor value changes significantly
        lerp_factor = 0.2 if abs(sensor_value - prev_sensor) > 0.1 else 0.1
        self.current_speed = self.current_speed * (1 - lerp_factor) + self.target_speed * lerp_factor
        self.current_emit_interval = self.current_emit_interval * (1 - lerp_factor) + self.target_emit_interval * lerp_factor
        
        # Update timing
        self.time_until_next_emit -= dt
        
        # If sensor value increased significantly, trigger immediate emission
        if sensor_value > 0.05 and (prev_sensor < 0.05 or sensor_value > prev_sensor + 0.3):
            # Immediate emission for new/significant forces
            num_particles = max(1, int(1 + sensor_value * 10))  # 1 to 11 particles
            self.time_until_next_emit = self.current_emit_interval
            return num_particles
        
        if self.time_until_next_emit <= 0.0:
            # Time to emit! Calculate how many particles based on sensor value
            # Higher sensor value = more particles per emission
            # Even with sensor_value=0, emit at least 1 particle occasionally
            if sensor_value > 0.01:
                num_particles = max(1, int(1 + sensor_value * 10))  # 1 to 11 particles
            else:
                num_particles = 0  # No emission when sensor is near zero
            
            # Reset timer (use current interval, not target, to prevent rapid oscillation)
            self.time_until_next_emit = self.current_emit_interval
            return num_particles
        
        return 0
    
    def get_position(self):
        """Get current emission position"""
        return self.position_func()
    
    def sample_velocity(self):
        """Sample random velocity within cone spread"""
        angle = (random.random() - 0.5) * self.spread
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = self.dir
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        # Use current speed (which is sensor-controlled)
        sp = self.current_speed * (0.75 + 0.5 * random.random())
        return (rx * sp, ry * sp)


class TouchScreenApp:
    """Main application class with particle blob and four emitters"""
    
    def __init__(self):
        """Initialize the application"""
        # Load configuration
        self.config = load_all_config()
        
        # Initialize Pygame
        pygame.init()
        pygame.mouse.set_visible(True)
        
        # Get screen resolution and calculate 9:16 aspect ratio dimensions
        display_config = self.config['display']
        screen_width, screen_height = self._calculate_9_16_resolution()
        
        # Set up display (OpenGL)
        display_flags = pygame.OPENGL | pygame.DOUBLEBUF
        if display_config['fullscreen']:
            display_flags |= pygame.FULLSCREEN
        pygame.display.set_mode((screen_width, screen_height), display_flags)
        self.screen = pygame.Surface((screen_width, screen_height), flags=pygame.SRCALPHA)
        
        pygame.display.set_caption("Raspberry Pi Touch App")
        
        # Initialize renderer for layered drawing
        self.renderer = Renderer((screen_width, screen_height), self.screen)
        
        # Initialize clock for FPS control
        self.clock = pygame.time.Clock()
        self.fps = display_config['fps']
        
        # FPS logging
        self.fps_log_enabled = True
        # Create logs directory if it doesn't exist
        logs_dir = Path(__file__).parent.parent.parent / "logs"
        logs_dir.mkdir(exist_ok=True)
        # Create timestamped log file
        timestamp = dt.now().strftime("%Y%m%d_%H%M%S")
        self.fps_log_path = logs_dir / f"fps_log_{timestamp}.txt"
        self.fps_log_interval = 1.0  # Log FPS every 1 second
        self.fps_log_last_time = 0.0
        self.fps_samples = []  # Store FPS samples for averaging
        self.frame_count = 0
        
        # Initialize Arduino interface
        self.arduino = ArduinoInterface()
        arduino_config = self.config.get('arduino', {})
        if arduino_config.get('serial', {}).get('auto_connect', True):
            self.arduino.connect()
        
        # Initialize physics space
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)  # We'll apply gravity manually to particles
        
        # Collision handler types
        self.PARTICLE_COLLISION_TYPE = 1
        self.BOUNDARY_COLLISION_TYPE = 2
        
        # Screen dimensions
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # Particle storage
        self.particles = []
        self.max_particles = 2200
        
        # Blob center position (center of screen)
        self.blob_center_x = screen_width // 2
        self.blob_center_y = screen_height // 2
        self.blob_radius = 80  # Initial blob radius
        
        # Create four emitters at the corners: upper left, upper right, lower left, lower right
        # Directions normalized for diagonal corners
        sqrt2_inv = 1.0 / math.sqrt(2)  # Normalization factor for diagonal vectors
        
        emitter_configs = [
            # (name, position_offset_multiplier, direction)
            ("upper_left", (-1, -1), (-sqrt2_inv, -sqrt2_inv)),    # Upper left corner
            ("upper_right", (1, -1), (sqrt2_inv, -sqrt2_inv)),     # Upper right corner
            ("lower_left", (-1, 1), (-sqrt2_inv, sqrt2_inv)),      # Lower left corner
            ("lower_right", (1, 1), (sqrt2_inv, sqrt2_inv))        # Lower right corner
        ]
        
        self.emitters = []
        for i, (name, pos_mult, direction) in enumerate(emitter_configs):
            # Position emitter inside blob at corner offset from center
            offset_distance = self.blob_radius * 0.6  # Emit from inside blob
            offset_x = pos_mult[0] * offset_distance
            offset_y = pos_mult[1] * offset_distance
            
            def make_position_func(ox=offset_x, oy=offset_y):
                def get_pos():
                    return (self.blob_center_x + ox, self.blob_center_y + oy)
                return get_pos
            
            emitter = Emitter(
                name=name,
                position_func=make_position_func(),
                direction=direction,
                spread_deg=EMISSION_SPREAD_DEG
            )
            self.emitters.append(emitter)
        
        # Sensor values (0.0 to 1.0) - one per emitter
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]
        
        # Force sequence management for testing
        self.force_sequences: Dict[str, Dict] = {}
        self.current_sequence: Optional[str] = None
        self.sequence_start_time: float = 0.0
        self.sequence_playing: bool = False
        self.active_forces_display: List[Dict] = []  # For display on screen
        
        # Load force sequences from JSON
        self._load_force_sequences()
        
        # Emitter mapping: [upper_left, upper_right, lower_left, lower_right]
        # Index corresponds to emitter index
        self.emitter_names = ["upper_left", "upper_right", "lower_left", "lower_right"]
        
        # Time tracking for sequences
        self.sequence_time = 0.0
        
        # Create screen boundaries
        self._create_screen_boundaries()
        
        # Initialize fonts
        self.font_large = pygame.font.Font(None, 72)
        self.font_medium = pygame.font.Font(None, 48)
        self.font_small = pygame.font.Font(None, 36)
        
        # Application state
        self.running = True
        self.theme = self.config['theme']
        
        # Close button state
        self.screen_clicked = False
        self.button_alpha = 255
        self.last_click_time = 0
        self.fade_delay = 2000
        self.fade_speed = 2
        
        # Time tracking
        self.last_update_time = pygame.time.get_ticks()
        
        # Performance profiling
        self.profile_enabled = True
        self.profile_data = {
            'update_time': [],
            'draw_time': [],
            'tile_draw_time': [],
            'sprite_draw_time': [],
            'num_tiles': [],
            'num_particles': []
        }
        
        # Close button properties
        self.button_size = 60
        self.button_padding = 20
        
        # Corridor scrolling effect - 3D perspective
        self.scroll_offset_z = 0.0  # Current Z scroll offset (increases over time, objects move backward)
        self.hallway_segments = []  # List of hallway segment positions for wireframe
        self._initialize_hallway_segments()
        
        # Initialize tile manager for hallway tiling (sprite surfaces will be set after loading)
        self.tile_manager = TileManager(tile_size=40.0, tile_color=(128, 128, 128))
        # Initial tile generation will happen in update() with proper screen dimensions
        
        # Load heart image
        self._load_heart_image()
        
        # Load sprite tiles and data
        self._load_sprite_tiles()
        
        print("TouchScreenApp initialized successfully")
    
    def _initialize_hallway_segments(self):
        """Initialize hallway segments for wireframe visualization with 3D perspective"""
        # Create initial hallway segments in Z-space that will scroll backward
        # Each segment is a decagon (10-sided polygon) representing a section of the corridor
        # Start with many segments to ensure good coverage
        num_segments = 30  # Number of initial segments (increased)
        # Calculate base radius for decagon (fits within screen with margin)
        base_radius = min(self.screen_width, self.screen_height) / 2 - 30
        
        for i in range(num_segments):
            # Segments start at near plane and extend into distance
            segment_z = NEAR_PLANE_Z + HALLWAY_SEGMENT_LENGTH * i
            self.hallway_segments.append({
                "z_depth": segment_z,  # Z position (depth) - increases as segments move backward
                "base_radius": base_radius,  # Base radius of decagon at near plane
                "segment_length": HALLWAY_SEGMENT_LENGTH  # Length of segment in Z
            })
    
    def _update_hallway_segments(self, dt):
        """Update hallway segments for scrolling effect in 3D Z-space"""
        scroll_delta_z = CORRIDOR_SCROLL_SPEED * dt
        
        # Move all segments backward in Z (increase Z = farther away)
        for segment in self.hallway_segments:
            segment["z_depth"] += scroll_delta_z
        
        # Remove segments that have scrolled too far back (beyond far plane)
        # Keep a large buffer - only remove if well beyond far plane
        # This ensures segments remain visible longer
        self.hallway_segments = [s for s in self.hallway_segments 
                                 if s["z_depth"] < FAR_PLANE_Z * 2]
        
        # Add new segments at the near plane if needed
        # Find the closest (smallest Z) segment
        if self.hallway_segments:
            min_z = min((s["z_depth"] for s in self.hallway_segments), default=NEAR_PLANE_Z)
        else:
            # If no segments exist (shouldn't happen, but handle it), reinitialize
            min_z = NEAR_PLANE_Z
            self._initialize_hallway_segments()
            return
        
        # Add segments at near plane until we have enough coverage
        # Ensure we always have segments covering from NEAR_PLANE_Z to well beyond visible range
        base_radius = min(self.screen_width, self.screen_height) / 2 - 30
        target_coverage = FAR_PLANE_Z - NEAR_PLANE_Z  # Cover entire visible Z range
        min_segments = 30  # Always maintain at least this many segments (increased for better coverage)
        
        # Keep adding segments until we have full coverage
        segments_added = 0
        max_segments_to_add = 100  # Safety limit per frame (increased)
        while (min_z > NEAR_PLANE_Z - HALLWAY_SEGMENT_LENGTH or len(self.hallway_segments) < min_segments) and segments_added < max_segments_to_add:
            new_segment = {
                "z_depth": min_z - HALLWAY_SEGMENT_LENGTH,
                "base_radius": base_radius,
                "segment_length": HALLWAY_SEGMENT_LENGTH
            }
            self.hallway_segments.append(new_segment)
            min_z = new_segment["z_depth"]
            segments_added += 1
            
            # Safety check to prevent infinite loop
            if min_z < NEAR_PLANE_Z - target_coverage * 4:
                break
    
    def _calculate_9_16_resolution(self):
        """Calculate resolution with 9:16 aspect ratio"""
        screen_info = pygame.display.Info()
        screen_width = screen_info.current_w
        screen_height = screen_info.current_h
        
        target_ratio = 9 / 16
        
        width_from_width = screen_width
        height_from_width = int(width_from_width / target_ratio)
        
        height_from_height = screen_height
        width_from_height = int(height_from_height * target_ratio)
        
        if height_from_width <= screen_height:
            final_width = width_from_width
            final_height = height_from_width
        else:
            final_width = width_from_height
            final_height = height_from_height
        
        print(f"Screen resolution: {screen_width}x{screen_height}")
        print(f"Calculated 9:16 resolution: {final_width}x{final_height}")
        
        return final_width, final_height
    
    def _create_screen_boundaries(self):
        """Create static boundaries at screen edges with hard, sticky collisions"""
        static = self.space.static_body
        margin = 30
        
        # Create boundary segments (top, bottom, left, right)
        segs = [
            pymunk.Segment(static, (margin, margin), (self.screen_width - margin, margin), 6),  # Top
            pymunk.Segment(static, (margin, self.screen_height - margin), 
                          (self.screen_width - margin, self.screen_height - margin), 6),  # Bottom
            pymunk.Segment(static, (margin, margin), (margin, self.screen_height - margin), 6),  # Left
            pymunk.Segment(static, (self.screen_width - margin, margin), 
                          (self.screen_width - margin, self.screen_height - margin), 6),  # Right
        ]
        
        for s in segs:
            # Extremely high friction for maximum stickiness
            s.friction = 5.0
            # Low elasticity for non-bouncy, hard collisions
            s.elasticity = 0.0
            # Set collision type for handlers
            s.collision_type = self.BOUNDARY_COLLISION_TYPE
            s.filter = pymunk.ShapeFilter(
                categories=0x4000,
                mask=0x8000
            )
        
        self.space.add(*segs)
        self.boundaries = segs
        
        # Store boundary info for flattening effect and manual collision enforcement
        self.boundary_segments = segs
        self.margin = margin
        
        # Store decagon boundary info for collision checking
        # Decagon is centered at vanishing point, with radius that fits screen
        self.decagon_center_x = self.screen_width / 2
        self.decagon_center_y = self.screen_height * VANISHING_POINT_Y
        self.decagon_radius = min(self.screen_width, self.screen_height) / 2 - margin
        
    
    def _load_force_sequences(self):
        """Load force sequences from JSON file for testing emitter behavior"""
        try:
            sequences_path = Path(__file__).parent.parent.parent / "config" / "force_sequences.json"
            with open(sequences_path, 'r') as f:
                data = json.load(f)
            
            sequences_data = data.get("sequences", [])
            for seq_data in sequences_data:
                name = seq_data.get("name")
                forces = seq_data.get("forces", [])
                
                if name:
                    self.force_sequences[name] = {
                        "name": name,
                        "forces": forces
                    }
                    print(f"Loaded force sequence for testing: {name} ({len(forces)} forces)")
        except Exception as e:
            print(f"Warning: Could not load force sequences for testing: {e}")
    
    def _load_heart_image(self):
        """Load and scale heart image to cover emitter area (event horizon)"""
        try:
            heart_path = Path(__file__).parent.parent.parent / "assets" / "images" / "heart.png"
            if heart_path.exists():
                # Load original heart image
                self.heart_image_original = pygame.image.load(str(heart_path)).convert_alpha()
                
                # Calculate size to cover emitter area
                # Emitters are at offset_distance = blob_radius * 0.6 from center
                offset_distance = self.blob_radius * 0.6
                # Diagonal distance from center to emitter
                diagonal_distance = math.sqrt(offset_distance * offset_distance * 2)
                # Size to cover all emitters (add some padding for full coverage)
                heart_size = int(diagonal_distance * 5.0)  # Scale to cover all emitters with margin (doubled)
                
                # Scale heart image to calculated size
                self.heart_image = pygame.transform.smoothscale(
                    self.heart_image_original, 
                    (heart_size, heart_size)
                )
                self.heart_size = heart_size
                print(f"Loaded heart image, scaled to {heart_size}x{heart_size} pixels")
            else:
                print(f"Warning: Heart image not found at {heart_path}")
                self.heart_image = None
                self.heart_size = 0
        except Exception as e:
            print(f"Warning: Could not load heart image: {e}")
            self.heart_image = None
            self.heart_size = 0
    
    def _load_sprite_tiles(self):
        """Load sprite sheet and tile data for hallway surfaces"""
        try:
            # Load sprite sheet image
            sprite_path = Path(__file__).parent.parent.parent / "assets" / "images" / "test_pipes.png"
            if sprite_path.exists():
                self.sprite_sheet = pygame.image.load(str(sprite_path)).convert_alpha()
                print(f"Loaded sprite sheet: {sprite_path}")
            else:
                print(f"Warning: Sprite sheet not found at {sprite_path}")
                self.sprite_sheet = None
                self.tile_data = []
                return
            
            # Load tile data JSON
            data_path = Path(__file__).parent.parent.parent / "assets" / "data" / "squares_data_unique.json"
            if data_path.exists():
                with open(data_path, 'r') as f:
                    self.tile_data = json.load(f)
                print(f"Loaded {len(self.tile_data)} tile definitions")
            else:
                print(f"Warning: Tile data not found at {data_path}")
                self.tile_data = []
                return
            
            # Pre-extract tile surfaces for faster rendering
            self.tile_surfaces = []
            for tile_info in self.tile_data:
                x = tile_info.get("sprite_x", 0)
                y = tile_info.get("sprite_y", 0)
                w = tile_info.get("sprite_width", 180)
                h = tile_info.get("sprite_height", 180)
                
                # Extract tile from sprite sheet
                tile_surface = pygame.Surface((w, h), pygame.SRCALPHA)
                tile_surface.blit(self.sprite_sheet, (0, 0), (x, y, w, h))
                self.tile_surfaces.append(tile_surface)
            
            print(f"Pre-extracted {len(self.tile_surfaces)} tile surfaces")
            
            # Update tile manager with sprite surfaces
            if hasattr(self, 'tile_manager'):
                self.tile_manager.sprite_surfaces = self.tile_surfaces
                print(f"Updated tile manager with {len(self.tile_surfaces)} sprite surfaces")
            
        except Exception as e:
            print(f"Warning: Could not load sprite tiles: {e}")
            self.sprite_sheet = None
            self.tile_data = []
            self.tile_surfaces = []
    
    def _disable_particle_collisions(self, particle):
        """
        Disable collisions for a flattened particle
        When flattened, disable all collisions (set mask to 0)
        Flattened particles never unflatten, so collisions remain disabled permanently
        """
        if "shape" not in particle:
            return
        
        shape = particle["shape"]
        # Disable all collisions (mask = 0 means collide with nothing)
        shape.filter = pymunk.ShapeFilter(
            categories=0x8000,
            mask=0x0000
        )
    
    def _check_flatten_from_force(self, particle, body, force_x, force_y, dt):
        """
        Check if force would push particle past decagon boundary BEFORE applying force
        If force would push past boundary, flatten the particle immediately
        """
        particle_radius = PARTICLE_RADIUS
        boundary_thickness = 6
        effective_radius = self.decagon_radius - particle_radius - boundary_thickness
        
        # Only check if particle is not already flattened
        if particle.get("state") == "flattened":
            return
        
        pos_x = float(body.position.x)
        pos_y = float(body.position.y)
        vel_x = float(body.velocity.x)
        vel_y = float(body.velocity.y)
        
        # Calculate predicted position after applying this force
        # acceleration = force / mass
        accel_x = force_x / PARTICLE_MASS
        accel_y = force_y / PARTICLE_MASS
        # Predicted velocity
        predicted_vel_x = vel_x + accel_x * dt
        predicted_vel_y = vel_y + accel_y * dt
        # Predicted position
        predicted_pos_x = pos_x + predicted_vel_x * dt
        predicted_pos_y = pos_y + predicted_vel_y * dt
        
        # Calculate distance from decagon center for current and predicted positions
        dx = pos_x - self.decagon_center_x
        dy = pos_y - self.decagon_center_y
        current_dist = math.sqrt(dx * dx + dy * dy)
        
        pred_dx = predicted_pos_x - self.decagon_center_x
        pred_dy = predicted_pos_y - self.decagon_center_y
        predicted_dist = math.sqrt(pred_dx * pred_dx + pred_dy * pred_dy)
        
        # Threshold for being "at" boundary (reduced to prevent premature flattening)
        at_boundary_threshold = 3.0  # pixels
        
        # Check if force would push particle past decagon boundary
        if current_dist >= effective_radius - at_boundary_threshold:
            if predicted_dist > effective_radius or (current_dist >= effective_radius and predicted_dist > current_dist):
                # Force would push past decagon boundary - flatten immediately!
                # Calculate normal vector (pointing outward from center)
                if current_dist > 1e-6:
                    normal_x = dx / current_dist
                    normal_y = dy / current_dist
                else:
                    normal_x = 1.0
                    normal_y = 0.0
                
                particle["state"] = "flattened"
                particle["flattened_against"] = "decagon"  # Store that it's against decagon
                particle["blocked_direction"] = (normal_x, normal_y)  # Normal pointing outward
                
                # Disable collisions for flattened particle (permanent state)
                self._disable_particle_collisions(particle)
                
                # Position particle exactly on boundary
                boundary_x = self.decagon_center_x + normal_x * effective_radius
                boundary_y = self.decagon_center_y + normal_y * effective_radius
                body.position = (boundary_x, boundary_y)
                
                # Zero velocity component towards boundary, keep tangential
                vel_dot_normal = vel_x * normal_x + vel_y * normal_y
                if vel_dot_normal > 0:  # Moving outward
                    vel_normal_x = vel_dot_normal * normal_x
                    vel_normal_y = vel_dot_normal * normal_y
                    body.velocity = (vel_x - vel_normal_x, vel_y - vel_normal_y)
                else:
                    body.velocity = (vel_x, vel_y)
                return
    
    def _check_flatten_from_position_and_velocity(self, particle, body, dt):
        """
        Backup check: if particle got past decagon boundary during physics step, flatten it
        This checks AFTER physics step when positions have been updated
        """
        particle_radius = PARTICLE_RADIUS
        boundary_thickness = 6
        effective_radius = self.decagon_radius - particle_radius - boundary_thickness
        
        # Only check if particle is not already flattened
        if particle.get("state") == "flattened":
            return
        
        pos_x = float(body.position.x)
        pos_y = float(body.position.y)
        vel_x = float(body.velocity.x)
        vel_y = float(body.velocity.y)
        
        # Check distance from decagon center
        dx = pos_x - self.decagon_center_x
        dy = pos_y - self.decagon_center_y
        dist_from_center = math.sqrt(dx * dx + dy * dy)
        
        # Check if particle is past decagon boundary
        if dist_from_center > effective_radius or (dist_from_center >= effective_radius - 3.0 and dist_from_center > effective_radius * 0.99):
            # Calculate normal vector (pointing outward from center)
            if dist_from_center > 1e-6:
                normal_x = dx / dist_from_center
                normal_y = dy / dist_from_center
            else:
                normal_x = 1.0
                normal_y = 0.0
            
            # Check if velocity is pushing outward
            vel_dot_normal = vel_x * normal_x + vel_y * normal_y
            
            if dist_from_center > effective_radius or (dist_from_center >= effective_radius - 3.0 and vel_dot_normal > 0):
                particle["state"] = "flattened"
                particle["flattened_against"] = "decagon"
                particle["blocked_direction"] = (normal_x, normal_y)
                
                # Disable collisions for flattened particle (permanent state)
                self._disable_particle_collisions(particle)
                
                # Position particle exactly on boundary
                boundary_x = self.decagon_center_x + normal_x * effective_radius
                boundary_y = self.decagon_center_y + normal_y * effective_radius
                body.position = (boundary_x, boundary_y)
                
                # Zero velocity component towards boundary, keep tangential
                if vel_dot_normal > 0:
                    vel_normal_x = vel_dot_normal * normal_x
                    vel_normal_y = vel_dot_normal * normal_y
                    body.velocity = (vel_x - vel_normal_x, vel_y - vel_normal_y)
                else:
                    body.velocity = (vel_x, vel_y)
                return
    
    def _maintain_flattened_position(self, particle):
        """Keep flattened particle exactly at decagon boundary position with strong friction"""
        particle_radius = PARTICLE_RADIUS
        boundary_thickness = 6
        effective_radius = self.decagon_radius - particle_radius - boundary_thickness
        body = particle["body"]
        pos_x = float(body.position.x)
        pos_y = float(body.position.y)
        flattened_against = particle.get("flattened_against")
        blocked_direction = particle.get("blocked_direction")
        
        # Strong friction coefficient for flattened particles (reduce tangential velocity significantly)
        FLATTENED_FRICTION = 0.3  # Keep only 30% of tangential velocity per frame (strong friction)
        
        # Snap to exact decagon boundary position and apply friction using blocked direction
        if flattened_against == "decagon" and blocked_direction:
            blocked_nx, blocked_ny = blocked_direction
            vel_x, vel_y = float(body.velocity.x), float(body.velocity.y)
            
            # Project velocity onto blocked direction (normal) and tangential
            vel_dot_blocked = vel_x * blocked_nx + vel_y * blocked_ny
            vel_normal_x = vel_dot_blocked * blocked_nx
            vel_normal_y = vel_dot_blocked * blocked_ny
            vel_tangential_x = vel_x - vel_normal_x
            vel_tangential_y = vel_y - vel_normal_y
            
            # Zero normal velocity, apply friction to tangential
            new_vel_x = vel_tangential_x * FLATTENED_FRICTION
            new_vel_y = vel_tangential_y * FLATTENED_FRICTION
            
            # Snap to exact decagon boundary position
            # Use the normal to position particle on boundary circle
            boundary_x = self.decagon_center_x + blocked_nx * effective_radius
            boundary_y = self.decagon_center_y + blocked_ny * effective_radius
            body.position = (boundary_x, boundary_y)
            
            body.velocity = (new_vel_x, new_vel_y)
    
    def _enforce_boundary_collisions(self):
        """Manually enforce hard boundary collisions and stickiness with decagon boundaries"""
        particle_radius = PARTICLE_RADIUS
        boundary_thickness = 6  # Boundary segment radius
        effective_radius = self.decagon_radius - particle_radius - boundary_thickness
        
        for p in self.particles:
            # Skip collision handling for flattened particles in their flattened direction
            # They've already been positioned and stopped, and remain flattened permanently
            if p["state"] == "flattened":
                # Keep flattened particles exactly at boundary
                if p["flattened_against"]:
                    self._maintain_flattened_position(p)
                continue
            body = p["body"]
            pos_x = float(body.position.x)
            pos_y = float(body.position.y)
            vel_x = float(body.velocity.x)
            vel_y = float(body.velocity.y)
            
            # Check distance from decagon center
            dx = pos_x - self.decagon_center_x
            dy = pos_y - self.decagon_center_y
            dist_from_center = math.sqrt(dx * dx + dy * dy)
            
            # Check if particle is outside decagon boundary
            if dist_from_center > effective_radius:
                # Particle is outside decagon - push it back inside
                overlap = dist_from_center - effective_radius
                if overlap > 0:
                    # Calculate normal vector pointing from center to particle (outward)
                    if dist_from_center > 1e-6:
                        normal_x = dx / dist_from_center
                        normal_y = dy / dist_from_center
                    else:
                        normal_x = 1.0
                        normal_y = 0.0
                    
                    # Push particle back inside decagon
                    push_distance = overlap * 2.5  # Stronger push
                    new_pos_x = pos_x - normal_x * push_distance
                    new_pos_y = pos_y - normal_y * push_distance
                    body.position = (new_pos_x, new_pos_y)
                    
                    # Apply stickiness: reduce velocity towards boundary
                    # Normal points outward, so we want to reduce velocity in that direction
                    vel_dot_normal = vel_x * normal_x + vel_y * normal_y
                    
                    # Calculate tangential velocity (sliding along boundary)
                    vel_normal_x = vel_dot_normal * normal_x
                    vel_normal_y = vel_dot_normal * normal_y
                    vel_tangential_x = vel_x - vel_normal_x
                    vel_tangential_y = vel_y - vel_normal_y
                    
                    if vel_dot_normal > 0:  # Moving outward (towards boundary)
                        # Extremely strong damping: 99% reduction of velocity towards boundary
                        # Also reduce tangential velocity by 90% to prevent sliding
                        new_vel_x = vel_x - vel_normal_x * 0.99 - vel_tangential_x * 0.90
                        new_vel_y = vel_y - vel_normal_y * 0.99 - vel_tangential_y * 0.90
                        
                        body.velocity = (new_vel_x, new_vel_y)
                    else:
                        # Particle is on or slightly inside boundary - strong tangential damping
                        # Reduce tangential sliding by 92% to create sticky effect
                        new_vel_x = vel_x - vel_tangential_x * 0.92
                        new_vel_y = vel_y - vel_tangential_y * 0.92
                        
                        body.velocity = (new_vel_x, new_vel_y)
                    
                    # Apply additional sticky force: pull particle slightly towards boundary
                    # This creates a "magnetic" sticky effect
                    sticky_force_strength = 50.0  # Force strength
                    sticky_force_x = normal_x * sticky_force_strength * PARTICLE_MASS
                    sticky_force_y = normal_y * sticky_force_strength * PARTICLE_MASS
                    body.apply_force_at_local_point((-sticky_force_x, -sticky_force_y), (0, 0))
    
    def _map_force_to_emitters(self, force_vec: Tuple[float, float]) -> List[float]:
        """
        Map a force vector to emitter sensor values
        Distributes the force across the four corner emitters based on direction
        
        Args:
            force_vec: (fx, fy) force vector
            
        Returns:
            List of 4 sensor values [upper_left, upper_right, lower_left, lower_right]
        """
        fx, fy = force_vec
        
        # Calculate force magnitude for intensity
        force_magnitude = math.sqrt(fx * fx + fy * fy)
        if force_magnitude < 0.001:
            return [0.0, 0.0, 0.0, 0.0]
        
        # Normalize magnitude to 0.0-1.0 range (assuming max force ~500)
        max_force = 500.0
        intensity = clamp(force_magnitude / max_force, 0.0, 1.0)
        
        # Normalize force direction
        norm_fx = fx / force_magnitude
        norm_fy = fy / force_magnitude
        
        # Map force direction to corner emitters using dot product approach
        # Each corner has a direction vector pointing from center
        # Upper left: (-sqrt2_inv, -sqrt2_inv)
        # Upper right: (sqrt2_inv, -sqrt2_inv)
        # Lower left: (-sqrt2_inv, sqrt2_inv)
        # Lower right: (sqrt2_inv, sqrt2_inv)
        sqrt2_inv = 1.0 / math.sqrt(2)
        corner_directions = [
            (-sqrt2_inv, -sqrt2_inv),   # Upper left
            (sqrt2_inv, -sqrt2_inv),    # Upper right
            (-sqrt2_inv, sqrt2_inv),    # Lower left
            (sqrt2_inv, sqrt2_inv)      # Lower right
        ]
        
        # Calculate dot products (how aligned force is with each corner direction)
        # Then convert to contributions (negative values become 0)
        contributions = []
        for corner_dir_x, corner_dir_y in corner_directions:
            # Dot product: how aligned the force is with this corner direction
            dot_product = norm_fx * corner_dir_x + norm_fy * corner_dir_y
            # Convert to contribution (0.0 to 1.0), prefer positive alignment
            contrib = max(0.0, dot_product)
            contributions.append(contrib)
        
        # Normalize contributions to sum to 1.0, then scale by intensity
        total_contrib = sum(contributions)
        if total_contrib > 0.001:
            contributions = [c / total_contrib for c in contributions]
        else:
            # If force points away from all corners (unlikely), distribute evenly
            contributions = [0.25, 0.25, 0.25, 0.25]
        
        # Return sensor values scaled by intensity
        return [c * intensity for c in contributions]
    
    def _play_force_sequence(self, sequence_name: str):
        """Start playing a force sequence for testing"""
        if sequence_name not in self.force_sequences:
            print(f"Warning: Force sequence '{sequence_name}' not found for testing")
            print(f"Available sequences: {list(self.force_sequences.keys())}")
            return
        
        # Initialize sequence time if not already initialized
        if self.sequence_time == 0.0:
            self.sequence_time = pygame.time.get_ticks() / 1000.0
        
        self.current_sequence = sequence_name
        self.sequence_start_time = self.sequence_time
        self.sequence_playing = True
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]  # Reset sensor values
        
        sequence = self.force_sequences[sequence_name]
        num_forces = len(sequence.get("forces", []))
        print(f"Playing force sequence '{sequence_name}' with {num_forces} forces")
    
    def _stop_force_sequence(self):
        """Stop currently playing force sequence"""
        self.sequence_playing = False
        self.current_sequence = None
    
    def _update_force_sequences(self, dt: float):
        """Update active force sequences and map them to emitter sensor values"""
        # Reset active forces display
        self.active_forces_display = []
        
        if not self.sequence_playing or not self.current_sequence:
            # Reset sensor values when no sequence is playing
            if not self.arduino.is_connected():
                self.sensor_values = [0.0, 0.0, 0.0, 0.0]
            return
        
        sequence = self.force_sequences[self.current_sequence]
        sequence_elapsed = self.sequence_time - self.sequence_start_time
        
        # Get active forces at current time
        active_forces = []
        for force_def in sequence["forces"]:
            force_time = force_def.get("time", 0.0)
            duration = force_def.get("duration", 0.0)
            fade_type = force_def.get("fade_type", "linear")
            
            # Check if this force should be active
            if force_time <= sequence_elapsed < force_time + duration:
                elapsed_in_force = sequence_elapsed - force_time
                active_forces.append({
                    "force_def": force_def,
                    "elapsed": elapsed_in_force,
                    "fade_type": fade_type
                })
        
        # Calculate fade factors and apply forces
        emitter_values = [0.0, 0.0, 0.0, 0.0]
        total_forward_push = 0.0  # Accumulate forward push from all active forces
        
        for active_force in active_forces:
            force_def = active_force["force_def"]
            elapsed = active_force["elapsed"]
            duration = force_def.get("duration", 0.0)
            fade_type = active_force["fade_type"]
            target = force_def.get("target", "unknown")
            original_force = force_def.get("force", [0, 0])
            
            # Calculate fade factor
            if duration <= 0.0:
                fade_factor = 1.0 if fade_type == "instant" else 0.0
            elif elapsed >= duration:
                fade_factor = 0.0
            else:
                progress = elapsed / duration
                if fade_type == "instant":
                    fade_factor = 1.0 if progress < 1.0 else 0.0
                elif fade_type == "linear":
                    fade_factor = 1.0 - progress
                elif fade_type == "exponential":
                    # Exponential decay
                    k = -math.log(0.01) / duration
                    fade_factor = math.exp(-k * elapsed)
                else:
                    fade_factor = 1.0 - progress
            
            # Get force vector
            force_vec = force_def.get("force", [0, 0])
            force_faded = (force_vec[0] * fade_factor, force_vec[1] * fade_factor)
            force_magnitude = math.sqrt(force_faded[0]**2 + force_faded[1]**2)
            
            # Get forward push value (0.0 to 1.0, optional)
            forward_push = force_def.get("forward_push", 0.0)
            # Apply fade to forward push as well
            forward_push_faded = forward_push * fade_factor
            total_forward_push += forward_push_faded
            
            # Store for display
            self.active_forces_display.append({
                "target": target,
                "force": force_faded,
                "magnitude": force_magnitude,
                "fade_factor": fade_factor,
                "forward_push": forward_push_faded,
                "elapsed": elapsed,
                "duration": duration
            })
            
            # Map to emitters
            mapped_values = self._map_force_to_emitters(force_faded)
            
            # Combine with existing values (sum to allow multiple forces)
            for i in range(4):
                emitter_values[i] += mapped_values[i]
        
        # Add forward push contribution to all emitter values
        # Forward push boosts all emitters equally
        forward_push_boost = clamp(total_forward_push, 0.0, 1.0)
        for i in range(4):
            emitter_values[i] += forward_push_boost * 0.5  # Add up to 0.5 boost from forward push
        
        # Clamp emitter values to 0.0-1.0 range
        for i in range(4):
            emitter_values[i] = clamp(emitter_values[i], 0.0, 1.0)
        
        # Update sensor values with force sequence values
        # Force sequences override Arduino when playing (for testing)
        if self.sequence_playing:
            # Store old values for debugging
            old_values = self.sensor_values.copy() if len(self.sensor_values) == 4 else [0.0, 0.0, 0.0, 0.0]
            self.sensor_values = emitter_values
            
            # Debug output when values change significantly
            if any(abs(emitter_values[i] - old_values[i]) > 0.05 for i in range(4)):
                print(f"Force sequence '{self.current_sequence}' active:")
                print(f"  Sensor values: {[f'{v:.3f}' for v in emitter_values]}")
                print(f"  Active forces: {len(active_forces)}")
                for force_info in self.active_forces_display:
                    print(f"    {force_info['target']}: force=({force_info['force'][0]:.1f}, {force_info['force'][1]:.1f}), mag={force_info['magnitude']:.1f}, fade={force_info['fade_factor']:.2f}")
        
        # Check if sequence is done
        max_time = max((f.get("time", 0.0) + f.get("duration", 0.0) 
                       for f in sequence["forces"]), default=0.0)
        if sequence_elapsed >= max_time + 0.1:  # Small buffer
            self._stop_force_sequence()
            if not self.arduino.is_connected():
                self.sensor_values = [0.0, 0.0, 0.0, 0.0]
    
    def _make_particle(self, x, y, vx, vy):
        """Create a particle at position with velocity"""
        body = pymunk.Body()
        body.position = (x, y)
        body.velocity = (vx, vy)
        body.mass = PARTICLE_MASS
        body.moment = pymunk.moment_for_circle(PARTICLE_MASS, 0, PARTICLE_RADIUS)
        
        shape = pymunk.Circle(body, PARTICLE_RADIUS)
        shape.friction = PARTICLE_FRICTION
        shape.elasticity = PARTICLE_ELASTICITY
        # Set collision type for handlers
        shape.collision_type = self.PARTICLE_COLLISION_TYPE
        shape.filter = pymunk.ShapeFilter(
            categories=0x8000,
            mask=0x8000 | 0x4000
        )
        
        self.space.add(body, shape)
        return body, shape
    
    def _parse_arduino_data(self, data):
        """
        Parse Arduino sensor data
        Expected format: "s1:s2:s3:s4" where each is a value 0-1023 or 0-1.0
        Or: "s1,s2,s3,s4" or just numbers separated by spaces
        """
        try:
            # Try different delimiters
            if ':' in data:
                parts = data.split(':')
            elif ',' in data:
                parts = data.split(',')
            else:
                parts = data.split()
            
            if len(parts) >= 4:
                values = []
                for part in parts[:4]:
                    try:
                        val = float(part.strip())
                        # Normalize to 0.0-1.0 range
                        # If value seems to be in 0-1023 range (analog), normalize it
                        if val > 1.0:
                            val = val / 1023.0
                        values.append(clamp(val, 0.0, 1.0))
                    except ValueError:
                        values.append(0.0)
                
                # Ensure we have exactly 4 values
                while len(values) < 4:
                    values.append(0.0)
                
                self.sensor_values = values[:4]
                return True
        except Exception as e:
            print(f"Error parsing Arduino data: {e}")
        
        return False
    
    def handle_events(self):
        """Process all pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_1:
                    # Play "jump" sequence
                    self._play_force_sequence("jump")
                elif event.key == pygame.K_2:
                    # Play "spin" sequence
                    self._play_force_sequence("spin")
                elif event.key == pygame.K_3:
                    # Play "wave" sequence
                    self._play_force_sequence("wave")
                elif event.key == pygame.K_4:
                    # Play "lean_left" sequence
                    self._play_force_sequence("lean_left")
                elif event.key == pygame.K_0:
                    # Stop all sequences
                    self._stop_force_sequence()
                    self.sensor_values = [0.0, 0.0, 0.0, 0.0]
                    print("Stopped force sequences and reset emitters")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_touch(event.pos, True)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.handle_touch(event.pos, False)
    
    def handle_touch(self, position, is_pressed):
        """Handle touch input"""
        if is_pressed:
            if self.screen_clicked and self.is_button_clicked(position):
                self.running = False
                return
            
            if not self.screen_clicked:
                self.screen_clicked = True
                self.button_alpha = 255
                self.last_click_time = pygame.time.get_ticks()
            else:
                self.button_alpha = 255
                self.last_click_time = pygame.time.get_ticks()
    
    def update(self):
        """Update application state"""
        current_time = pygame.time.get_ticks()
        dt = (current_time - self.last_update_time) / 1000.0
        self.last_update_time = current_time
        dt = clamp(dt, 0.0, 1.0 / 20.0)
        
        # Update corridor scroll offset in Z (everything moves backward in depth)
        self.scroll_offset_z += CORRIDOR_SCROLL_SPEED * dt
        
        # Update hallway segments for wireframe visualization
        self._update_hallway_segments(dt)
        
        # Update tiles for hallway segments (generate in screen space like wireframe)
        self.tile_manager.update_tiles_for_segments(
            self.hallway_segments,
            self.screen_width, self.screen_height,
            PERSPECTIVE_FOV, VANISHING_POINT_Y,
            generate_floor=True
        )
        
        # Update sequence time (in seconds)
        self.sequence_time = current_time / 1000.0
        
        # Update force sequences (maps forces to emitter sensor values)
        self._update_force_sequences(dt)
        
        # Read data from Arduino if connected and no sequence is playing
        # Force sequences override Arduino data when playing (for testing)
        if self.arduino.is_connected() and not self.sequence_playing:
            data = self.arduino.read_data()
            if data:
                self._parse_arduino_data(data)
        
        # Update emitters and emit particles
        for i, emitter in enumerate(self.emitters):
            sensor_val = self.sensor_values[i] if i < len(self.sensor_values) else 0.0
            num_particles = emitter.update(dt, sensor_val)
            
            # Emit particles
            for _ in range(num_particles):
                if len(self.particles) >= self.max_particles:
                    break
                
                try:
                    pos = emitter.get_position()
                    vx, vy = emitter.sample_velocity()
                    px, py = pos
                    px += random.uniform(-3, 3)
                    py += random.uniform(-3, 3)
                    
                    body, shape = self._make_particle(px, py, vx, vy)
                    self.particles.append({
                        "body": body, 
                        "shape": shape, 
                        "age": 0.0,
                        "z_depth": NEAR_PLANE_Z,  # Start at near plane
                        "state": "normal",  # "normal" or "flattened"
                        "flattened_against": None,  # "top", "bottom", "left", "right", or None
                        "blocked_direction": None  # Normal vector (x, y) of blocked direction, or None
                    })
                except Exception as e:
                    print(f"Error emitting particle from {emitter.name}: {e}")
        
        # Build spatial hash for neighbor interactions
        positions = [(p["body"].position.x, p["body"].position.y) for p in self.particles]
        grid = spatial_hash(positions)
        
        # Apply fluid forces (pressure + viscosity + cohesion)
        for i, p in enumerate(self.particles):
            bi = p["body"]
            xi, yi = float(bi.position.x), float(bi.position.y)
            vix, viy = float(bi.velocity.x), float(bi.velocity.y)
            
            cxi = int(xi // CELL)
            cyi = int(yi // CELL)
            
            fx = 0.0
            fy = 0.0
            
            for cell in neighbor_cells(cxi, cyi):
                for j in grid.get(cell, []):
                    if j == i:
                        continue
                    bj = self.particles[j]["body"]
                    xj, yj = float(bj.position.x), float(bj.position.y)
                    dx = xi - xj
                    dy = yi - yj
                    r2 = dx * dx + dy * dy
                    if r2 <= 1e-9:
                        continue
                    if r2 > NEIGHBOR_RADIUS * NEIGHBOR_RADIUS:
                        continue
                    
                    r = math.sqrt(r2)
                    
                    # Cohesion
                    if r > REST_DIST * COHESION_START and r < REST_DIST * COHESION_END:
                        nx = dx / r
                        ny = dy / r
                        t = (r - REST_DIST * COHESION_START) / (REST_DIST * (COHESION_END - COHESION_START))
                        pull = (1.0 - abs(2.0 * t - 1.0))
                        strength = pull * COHESION
                        fx -= nx * strength
                        fy -= ny * strength
                    
                    # Pressure
                    overlap = (REST_DIST - r)
                    if overlap > 0:
                        nx = dx / r
                        ny = dy / r
                        strength = (overlap / REST_DIST) * PRESSURE
                        fx += nx * strength
                        fy += ny * strength
                    
                    # Viscosity
                    vjx, vjy = float(bj.velocity.x), float(bj.velocity.y)
                    rvx = vjx - vix
                    rvy = vjy - viy
                    fx += rvx * VISCOSITY
                    fy += rvy * VISCOSITY
            
            # Calculate gravity force once
            gravity_x, gravity_y = GRAVITY
            gravity_force_x = gravity_x * PARTICLE_MASS
            gravity_force_y = gravity_y * PARTICLE_MASS
            
            # Check if normal particle should flatten BEFORE applying forces
            # This checks if the combined force (fluid + gravity) would push it past boundary
            if p.get("state") != "flattened":
                # Calculate total force including gravity
                total_force_x = fx + gravity_force_x
                total_force_y = fy + gravity_force_y
                
                # Check if this force would push particle past boundary
                self._check_flatten_from_force(p, bi, total_force_x, total_force_y, dt)
            
            # Filter forces based on flattened state
            # If particle is now flattened (from check above), remove forces in blocked direction
            # and apply friction to tangential forces
            if p.get("state") == "flattened" and p.get("blocked_direction"):
                blocked_nx, blocked_ny = p["blocked_direction"]
                FLATTENED_FRICTION = 0.3  # Strong friction - reduce tangential forces
                
                # Project fluid force onto blocked direction (normal) and tangential
                force_dot_blocked = fx * blocked_nx + fy * blocked_ny
                force_normal_x = force_dot_blocked * blocked_nx
                force_normal_y = force_dot_blocked * blocked_ny
                force_tangential_x = fx - force_normal_x
                force_tangential_y = fy - force_normal_y
                
                # Remove normal force, apply friction to tangential force
                if force_dot_blocked > 0:  # Pushing towards blocked boundary
                    fx = force_tangential_x * FLATTENED_FRICTION
                    fy = force_tangential_y * FLATTENED_FRICTION
                else:
                    # Not pushing towards boundary, just apply friction to tangential
                    fx = force_tangential_x * FLATTENED_FRICTION + force_normal_x
                    fy = force_tangential_y * FLATTENED_FRICTION + force_normal_y
                
                # Also filter gravity - cannot push in blocked direction, apply friction to tangential
                gravity_dot_blocked = gravity_force_x * blocked_nx + gravity_force_y * blocked_ny
                gravity_normal_x = gravity_dot_blocked * blocked_nx
                gravity_normal_y = gravity_dot_blocked * blocked_ny
                gravity_tangential_x = gravity_force_x - gravity_normal_x
                gravity_tangential_y = gravity_force_y - gravity_normal_y
                
                # Remove normal component, apply friction to tangential
                if gravity_dot_blocked > 0:  # Gravity pushing towards blocked boundary
                    gravity_force_x = gravity_tangential_x * FLATTENED_FRICTION
                    gravity_force_y = gravity_tangential_y * FLATTENED_FRICTION
                else:
                    # Not pushing towards boundary, just apply friction to tangential
                    gravity_force_x = gravity_tangential_x * FLATTENED_FRICTION + gravity_normal_x
                    gravity_force_y = gravity_tangential_y * FLATTENED_FRICTION + gravity_normal_y
                
                # Also apply friction to velocity component (reduce tangential velocity)
                vel_x, vel_y = float(bi.velocity.x), float(bi.velocity.y)
                vel_dot_blocked = vel_x * blocked_nx + vel_y * blocked_ny
                vel_normal_x = vel_dot_blocked * blocked_nx
                vel_normal_y = vel_dot_blocked * blocked_ny
                vel_tangential_x = vel_x - vel_normal_x
                vel_tangential_y = vel_y - vel_normal_y
                
                # Zero normal velocity, apply friction to tangential
                new_vel_x = vel_tangential_x * FLATTENED_FRICTION
                new_vel_y = vel_tangential_y * FLATTENED_FRICTION
                bi.velocity = (new_vel_x, new_vel_y)
            
            # Apply fluid forces (pressure, viscosity, cohesion)
            bi.apply_force_at_local_point((fx, fy), (0, 0))
            
            # Apply gravity manually (force = mass * acceleration)
            bi.apply_force_at_local_point((gravity_force_x, gravity_force_y), (0, 0))
        
        # Step physics (integrates forces into velocities and positions)
        self.space.step(dt)
        
        # Apply corridor scrolling (matches decagon movement rate)
        scroll_delta_z = CORRIDOR_SCROLL_SPEED * dt
        for p in self.particles:
            current_z = p.get("z_depth", NEAR_PLANE_Z)
            
            # Apply scrolling (always pushes backward at same rate as decagons)
            new_z = current_z + scroll_delta_z
            
            # Clamp Z to reasonable bounds
            p["z_depth"] = clamp(new_z, NEAR_PLANE_Z - 200, FAR_PLANE_Z)
        
        # Check for particles that should flatten AFTER physics step (backup check)
        # This catches any particles that got pushed past boundaries during physics step
        for p in self.particles:
            if p.get("state") != "flattened":
                self._check_flatten_from_position_and_velocity(p, p["body"], dt)
        
        # Enforce hard boundary collisions and stickiness (manual collision handling)
        # Flattened particles are handled separately - they skip normal collision
        self._enforce_boundary_collisions()
        
        # Apply global damping and cleanup
        dead = []
        for idx, p in enumerate(self.particles):
            p["age"] += dt
            b = p["body"]
            vx, vy = float(b.velocity.x), float(b.velocity.y)
            
            # Apply extra friction to flattened particles
            if p.get("state") == "flattened" and p.get("blocked_direction"):
                blocked_nx, blocked_ny = p["blocked_direction"]
                FLATTENED_FRICTION = 0.3
                
                # Project velocity onto normal and tangential
                vel_dot_blocked = vx * blocked_nx + vy * blocked_ny
                vel_normal_x = vel_dot_blocked * blocked_nx
                vel_normal_y = vel_dot_blocked * blocked_ny
                vel_tangential_x = vx - vel_normal_x
                vel_tangential_y = vy - vel_normal_y
                
                # Apply strong friction to tangential velocity
                new_vx = vel_tangential_x * FLATTENED_FRICTION
                new_vy = vel_tangential_y * FLATTENED_FRICTION
                b.velocity = (new_vx, new_vy)
            else:
                # Normal damping for non-flattened particles
                b.velocity = (vx * DAMPING, vy * DAMPING)
            
            # Remove old particles (greatly extended lifetime)
            if p["age"] > PARTICLE_LIFETIME:
                dead.append(idx)
                continue
            
            # Remove particles that are too far back in Z (beyond far plane)
            z_depth = p.get("z_depth", NEAR_PLANE_Z)
            if z_depth > FAR_PLANE_Z:
                dead.append(idx)
                continue
            
            # Also remove particles outside screen bounds (2D check)
            x, y = float(b.position.x), float(b.position.y)
            margin = 200
            if x < -margin or x > self.screen_width + margin or \
               y < -margin or y > self.screen_height + margin:
                dead.append(idx)
        
        # Remove dead particles
        for idx in reversed(dead):
            self.space.remove(self.particles[idx]["shape"], self.particles[idx]["body"])
            self.particles.pop(idx)
        
        # Handle button fading
        if self.screen_clicked:
            time_since_click = current_time - self.last_click_time
            if time_since_click > self.fade_delay:
                self.button_alpha = max(0, self.button_alpha - self.fade_speed)
    
    def is_button_clicked(self, position):
        """Check if the close button was clicked"""
        if not self.screen_clicked or self.button_alpha <= 0:
            return False
        
        screen_width = self.screen.get_width()
        button_x = screen_width - self.button_size - self.button_padding
        button_y = self.button_padding
        
        button_rect = pygame.Rect(button_x, button_y, self.button_size, self.button_size)
        return button_rect.collidepoint(position)
    
    def get_button_rect(self):
        """Get the rectangle for the close button"""
        screen_width = self.screen.get_width()
        button_x = screen_width - self.button_size - self.button_padding
        button_y = self.button_padding
        return pygame.Rect(button_x, button_y, self.button_size, self.button_size)
    
    def _get_hallway_wireframe_draw_func(self):
        """Get a draw function for wireframe hallway segments"""
        def draw_func(screen):
            """Draw wireframe hallway segments as decagons with 3D perspective projection"""
            # Use brighter color for better visibility
            wireframe_color = (120, 140, 180)  # Brighter blue-gray wireframe color
            line_width = 2
            
            # Safety check: if no segments, reinitialize
            if not self.hallway_segments:
                self._initialize_hallway_segments()
            
            # Draw each hallway segment as a wireframe decagon with perspective
            # Sort segments by Z-depth to draw back-to-front (though with wireframe it doesn't matter much)
            sorted_segments = sorted(self.hallway_segments, key=lambda s: s["z_depth"])
            
            segments_drawn = 0
            for segment in sorted_segments:
                z_depth = segment["z_depth"]
                
                # Only skip segments that are definitely too far back (beyond far plane)
                # Keep drawing segments even if they're slightly beyond, for smooth transition
                # Use a much larger buffer to keep segments visible longer
                if z_depth > FAR_PLANE_Z * 1.5:
                    continue
                
                # Get perspective scale for this depth
                scale = get_perspective_scale(z_depth)
                
                # Relaxed visibility check - only skip if scale is extremely small
                # This allows very distant segments to still be drawn (they'll be tiny but visible)
                if scale < 0.001:
                    continue
                
                # Calculate scaled radius (decagon gets smaller as it recedes)
                scaled_radius = segment["base_radius"] * scale
                
                # Relaxed radius check - allow very small decagons to be drawn
                # Even tiny decagons contribute to the tunnel effect
                if scaled_radius < 1.0:
                    continue
                
                # Center of screen (vanishing point)
                center_x = self.screen_width / 2
                center_y = self.screen_height * VANISHING_POINT_Y
                
                # Generate decagon vertices
                vertices = generate_decagon_vertices(center_x, center_y, scaled_radius)
                
                # Draw decagon wireframe (connect vertices)
                num_vertices = len(vertices)
                for i in range(num_vertices):
                    v1 = vertices[i]
                    v2 = vertices[(i + 1) % num_vertices]
                    
                    # Clamp vertices to screen bounds for drawing
                    v1_x = max(0, min(self.screen_width, int(v1[0])))
                    v1_y = max(0, min(self.screen_height, int(v1[1])))
                    v2_x = max(0, min(self.screen_width, int(v2[0])))
                    v2_y = max(0, min(self.screen_height, int(v2[1])))
                    
                    # Draw line if it's not completely off-screen
                    # Allow lines that are partially visible
                    if not ((v1_x == 0 and v2_x == 0 and v1_x == v2_x) or 
                            (v1_x == self.screen_width and v2_x == self.screen_width and v1_x == v2_x) or
                            (v1_y == 0 and v2_y == 0 and v1_y == v2_y) or 
                            (v1_y == self.screen_height and v2_y == self.screen_height and v1_y == v2_y)):
                        pygame.draw.line(screen, wireframe_color, 
                                       (v1_x, v1_y), 
                                       (v2_x, v2_y), line_width)
                
                segments_drawn += 1
        
        return draw_func
    
    def draw(self):
        """Render the application using the renderer"""
        # Clear screen
        bg_color = self.theme['background_color']
        self.renderer.clear(tuple(bg_color))
        
        # Draw title
        title_text = self.font_large.render(
            "Raspberry Pi Touch App",
            True,
            tuple(self.theme['text_color'])
        )
        title_rect = title_text.get_rect(center=(self.screen.get_width() // 2, 100))
        self.renderer.add_surface_rect(RenderLayer.UI_TEXT, title_text, title_rect)
        
        # Draw Arduino connection status
        arduino_status = "Connected" if self.arduino.is_connected() else "Disconnected"
        status_color = (0, 255, 0) if self.arduino.is_connected() else (255, 0, 0)
        status_text = self.font_medium.render(
            f"Arduino: {arduino_status}",
            True,
            status_color
        )
        status_rect = status_text.get_rect(center=(self.screen.get_width() // 2, 200))
        self.renderer.add_surface_rect(RenderLayer.UI_TEXT, status_text, status_rect)
        
        # Draw current sequence status and active forces
        y_offset = 250
        if self.sequence_playing and self.current_sequence:
            sequence_text = self.font_medium.render(
                f"Playing: {self.current_sequence}",
                True,
                (0, 255, 255)  # Cyan color
            )
            sequence_rect = sequence_text.get_rect(center=(self.screen.get_width() // 2, y_offset))
            self.renderer.add_surface_rect(RenderLayer.UI_TEXT, sequence_text, sequence_rect)
            
            # Draw active forces
            y_offset = 290
            if self.active_forces_display:
                forces_label = self.font_small.render("Active Forces:", True, (255, 255, 0))  # Yellow
                forces_rect = forces_label.get_rect(center=(self.screen.get_width() // 2, y_offset))
                self.renderer.add_surface_rect(RenderLayer.UI_TEXT, forces_label, forces_rect)
                y_offset += 30
                
                for force_info in self.active_forces_display:
                    target = force_info["target"]
                    fx, fy = force_info["force"]
                    mag = force_info["magnitude"]
                    fade = force_info["fade_factor"]
                    elapsed = force_info["elapsed"]
                    duration = force_info["duration"]
                    
                    # Color based on fade factor (green = full, red = faded)
                    color = (int(255 * (1 - fade)), int(255 * fade), 0)
                    
                    force_text = self.font_small.render(
                        f"{target}: ({fx:.0f}, {fy:.0f}) mag={mag:.0f} fade={fade:.2f}",
                        True,
                        color
                    )
                    force_rect = force_text.get_rect(center=(self.screen.get_width() // 2, y_offset))
                    self.renderer.add_surface_rect(RenderLayer.UI_TEXT, force_text, force_rect)
                    y_offset += 25
            else:
                no_forces_text = self.font_small.render("No active forces", True, (128, 128, 128))
                no_forces_rect = no_forces_text.get_rect(center=(self.screen.get_width() // 2, y_offset))
                self.renderer.add_surface_rect(RenderLayer.UI_TEXT, no_forces_text, no_forces_rect)
                y_offset += 30
            
            y_offset += 10
        else:
            y_offset = 250
        
        # Draw sensor values with emitter names
        emitter_labels = ["Upper Left", "Upper Right", "Lower Left", "Lower Right"]
        for i, (sensor_val, label) in enumerate(zip(self.sensor_values, emitter_labels)):
            # Color intensity based on sensor value
            intensity = int(100 + sensor_val * 155)
            color = (intensity, min(255, intensity // 2 + 100), min(255, intensity // 3 + 100))
            
            sensor_text = self.font_small.render(
                f"{label}: {sensor_val:.2f}",
                True,
                color
            )
            sensor_rect = sensor_text.get_rect(center=(self.screen.get_width() // 2, y_offset))
            self.renderer.add_surface_rect(RenderLayer.UI_TEXT, sensor_text, sensor_rect)
            y_offset += 35
        
        # Draw instructions
        instructions_y = y_offset + 20
        if not self.arduino.is_connected():
            instructions = [
                "Press 1-4 to test force sequences",
                "Press 0 to stop/reset",
                "1=Jump, 2=Spin, 3=Wave, 4=Lean Left"
            ]
            for instruction in instructions:
                inst_text = self.font_small.render(
                    instruction,
                    True,
                    (200, 200, 200)  # Light gray
                )
                inst_rect = inst_text.get_rect(center=(self.screen.get_width() // 2, instructions_y))
                self.renderer.add_surface_rect(RenderLayer.UI_TEXT, inst_text, inst_rect)
                instructions_y += 30
        
        # Draw FPS if enabled
        if self.config['debug']['show_fps']:
            fps_text = self.font_small.render(
                f"FPS: {int(self.clock.get_fps())}",
                True,
                tuple(self.theme['text_color'])
            )
            self.renderer.add_surface(RenderLayer.UI_TEXT, fps_text, (10, 10))
        
        # Draw particle count and flattened count
        flattened_count = sum(1 for p in self.particles if p.get("state") == "flattened")
        particle_text = self.font_small.render(
            f"Particles: {len(self.particles)} ({flattened_count} flattened)",
            True,
            tuple(self.theme['text_color'])
        )
        self.renderer.add_surface(RenderLayer.UI_TEXT, particle_text, (10, 50))
        
        # Draw available sequences info
        if len(self.force_sequences) > 0:
            seq_text = self.font_small.render(
                f"Sequences: {', '.join(self.force_sequences.keys())}",
                True,
                (150, 150, 150)  # Gray
            )
            self.renderer.add_surface(RenderLayer.UI_TEXT, seq_text, (10, self.screen_height - 80))
        
        # Draw hallway tiles (before wireframe so it appears behind)
        tile_draw_func = self.tile_manager.get_draw_function(
            self.screen_width, self.screen_height,
            PERSPECTIVE_FOV, VANISHING_POINT_Y,
            NEAR_PLANE_Z, FAR_PLANE_Z,
            font=getattr(self, 'font', None) or getattr(self, 'font_small', None)
        )
        self.renderer.add_draw_operation(RenderLayer.TILES, tile_draw_func)
        
        # Draw hallway wireframe (before particles so it appears behind)
        self.renderer.add_draw_operation(RenderLayer.WIREFRAME, self._get_hallway_wireframe_draw_func())
        
        # Draw particles using metaball rendering with flattening near decagon boundaries
        if len(self.particles) > 0:
            def draw_particles_func(screen):
                draw_metaballs(screen, self.particles, self.screen_width, self.screen_height, 
                              boundaries=self.boundary_segments if hasattr(self, 'boundary_segments') else None,
                              margin=30,
                              decagon_center_x=self.decagon_center_x if hasattr(self, 'decagon_center_x') else None,
                              decagon_center_y=self.decagon_center_y if hasattr(self, 'decagon_center_y') else None,
                              decagon_radius=self.decagon_radius if hasattr(self, 'decagon_radius') else None)
            self.renderer.add_draw_operation(RenderLayer.PARTICLES, draw_particles_func)
        
        # Draw heart image over emitter area (event horizon)
        if hasattr(self, 'heart_image') and self.heart_image is not None:
            heart_rect = self.heart_image.get_rect()
            heart_rect.center = (self.blob_center_x, self.blob_center_y)
            self.renderer.add_surface_rect(RenderLayer.HEART_IMAGE, self.heart_image, heart_rect)
        
        # Draw emitter indicators (small circles showing emitter positions)
        def draw_emitter_indicators_func(screen):
            for emitter in self.emitters:
                pos = emitter.get_position()
                # Draw a small circle at emitter position
                # Color intensity based on sensor value
                sensor_idx = self.emitters.index(emitter)
                sensor_val = self.sensor_values[sensor_idx] if sensor_idx < len(self.sensor_values) else 0.0
                intensity = int(100 + sensor_val * 155)
                color = (intensity, intensity // 2, intensity // 3)
                pygame.draw.circle(screen, color, (int(pos[0]), int(pos[1])), 8)
        self.renderer.add_draw_operation(RenderLayer.EMITTER_INDICATORS, draw_emitter_indicators_func)
        
        # Draw close button if screen has been clicked and button is visible
        if self.screen_clicked and self.button_alpha > 0:
            button_surface, button_rect = self._get_close_button_surface()
            self.renderer.add_surface_rect(RenderLayer.CLOSE_BUTTON, button_surface, button_rect)
        
        # Render everything (this flips the display)
        self.renderer.render()
    
    def _get_close_button_surface(self):
        """Get the close button surface"""
        button_rect = self.get_button_rect()
        button_surface = pygame.Surface((self.button_size, self.button_size), pygame.SRCALPHA)
        
        button_color = (200, 50, 50, self.button_alpha)
        pygame.draw.rect(button_surface, button_color, (0, 0, self.button_size, self.button_size), border_radius=10)
        
        line_color = (255, 255, 255, self.button_alpha)
        line_width = 4
        margin = 15
        pygame.draw.line(
            button_surface, 
            line_color, 
            (margin, margin), 
            (self.button_size - margin, self.button_size - margin), 
            line_width
        )
        pygame.draw.line(
            button_surface, 
            line_color, 
            (self.button_size - margin, margin), 
            (margin, self.button_size - margin), 
            line_width
        )
        
        return button_surface, button_rect
    
    def _log_fps(self, current_time: float, update_time: float = 0, draw_time: float = 0, frame_time: float = 0):
        """Log FPS to file with performance breakdown"""
        if not self.fps_log_enabled:
            return
        
        # Get current FPS
        current_fps = self.clock.get_fps()
        
        # Collect sample
        self.fps_samples.append(current_fps)
        self.frame_count += 1
        
        # Store performance data
        if self.profile_enabled:
            if update_time > 0:
                self.profile_data['update_time'].append(update_time)
            if draw_time > 0:
                self.profile_data['draw_time'].append(draw_time)
        
        # Log periodically (every fps_log_interval seconds)
        elapsed = current_time - self.fps_log_last_time
        if elapsed >= self.fps_log_interval:
            # Calculate statistics
            if self.fps_samples:
                avg_fps = sum(self.fps_samples) / len(self.fps_samples)
                min_fps = min(self.fps_samples)
                max_fps = max(self.fps_samples)
                
                # Calculate average times
                avg_update = sum(self.profile_data['update_time']) / len(self.profile_data['update_time']) if self.profile_data['update_time'] else 0
                avg_draw = sum(self.profile_data['draw_time']) / len(self.profile_data['draw_time']) if self.profile_data['draw_time'] else 0
                max_update = max(self.profile_data['update_time']) if self.profile_data['update_time'] else 0
                max_draw = max(self.profile_data['draw_time']) if self.profile_data['draw_time'] else 0
                
                # Write to log file
                try:
                    with open(self.fps_log_path, 'a') as f:
                        timestamp = dt.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        f.write(f"{timestamp} | FPS: {current_fps:.2f} | "
                               f"Avg: {avg_fps:.2f} | Min: {min_fps:.2f} | Max: {max_fps:.2f} | "
                               f"Frames: {self.frame_count} | "
                               f"Update: {avg_update:.1f}ms (max: {max_update:.1f}ms) | "
                               f"Draw: {avg_draw:.1f}ms (max: {max_draw:.1f}ms)\n")
                except Exception as e:
                    print(f"Warning: Could not write to FPS log: {e}")
            
            # Reset for next interval
            self.fps_samples = []
            self.profile_data['update_time'] = []
            self.profile_data['draw_time'] = []
            self.fps_log_last_time = current_time
    
    def run(self):
        """Main application loop"""
        print("Starting application...")
        print(f"FPS logging enabled: {self.fps_log_enabled}")
        if self.fps_log_enabled:
            print(f"FPS log file: {self.fps_log_path}")
            # Initialize log file with header
            try:
                with open(self.fps_log_path, 'w') as f:
                    f.write("FPS Log - Started at " + dt.now().strftime("%Y-%m-%d %H:%M:%S") + "\n")
                    f.write("=" * 120 + "\n")
                    f.write("Timestamp | FPS | Avg | Min | Max | Frames | Update(ms) | Draw(ms)\n")
                    f.write("=" * 120 + "\n")
            except Exception as e:
                print(f"Warning: Could not initialize FPS log file: {e}")
        
        start_time = pygame.time.get_ticks() / 1000.0
        self.fps_log_last_time = start_time
        
        while self.running:
            frame_start = pygame.time.get_ticks()
            
            self.handle_events()
            update_start = pygame.time.get_ticks()
            self.update()
            update_time = pygame.time.get_ticks() - update_start
            
            draw_start = pygame.time.get_ticks()
            self.draw()
            draw_time = pygame.time.get_ticks() - draw_start
            
            self.clock.tick(self.fps)
            
            # Log FPS and performance data
            current_time = pygame.time.get_ticks() / 1000.0
            frame_time = pygame.time.get_ticks() - frame_start
            self._log_fps(current_time, update_time, draw_time, frame_time)
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        if self.arduino.is_connected():
            self.arduino.disconnect()
        pygame.quit()


if __name__ == "__main__":
    app = TouchScreenApp()
    app.run()
