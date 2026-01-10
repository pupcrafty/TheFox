#!/usr/bin/env python3
"""
Fluid particle emitter module for stick figure
Emits particles from multiple emitters attached to stick figure body parts
"""

import math
import random
import numpy as np
import pygame
import pymunk
from collections import defaultdict


# ----------------------------
# Configuration
# ----------------------------
PARTICLE_RADIUS = 4
PARTICLE_MASS = 0.25
PARTICLE_FRICTION = 0.2
PARTICLE_ELASTICITY = 0.0

# Particle physics
GRAVITY = (0, 1200)
DAMPING = 0.995  # Global velocity damping per step

# Fluid forces
VISCOSITY = 0.10  # Higher => gooey
PRESSURE = 1800.0  # Higher => stiffer / more incompressible
REST_DIST = PARTICLE_RADIUS * 2.2  # Preferred spacing
NEIGHBOR_RADIUS = REST_DIST * 2.2  # Interaction distance

# Cohesion (surface tension)
COHESION = 260.0
COHESION_START = 0.9  # Start pulling at ~0.9 * REST_DIST
COHESION_END = 2.0  # Stop pulling at ~2.0 * REST_DIST

# Spatial hash cell size (close to neighbor radius)
CELL = int(NEIGHBOR_RADIUS)

# Rendering
RENDER_SCALE = 4  # Render at 1/4 resolution then scale up (speed)
ISO_THRESHOLD = 1.15  # Higher => thinner liquid, lower => fatter
FIELD_STRENGTH = 70.0  # Higher => blobs merge more
FIELD_SOFTEN = 3.0  # Avoids singularities near particle centers

# Emission timing
MIN_EMIT_INTERVAL = 0.4  # Minimum seconds between emissions
MAX_EMIT_INTERVAL = 1.4  # Maximum seconds between emissions

# Emission properties
PARTICLE_SPEED = 520  # Initial speed of emitted particles
EMISSION_SPREAD_DEG = 18  # Cone spread in degrees


# ----------------------------
# Helper Functions
# ----------------------------
def clamp(x, a, b):
    """Clamp value between a and b"""
    return a if x < a else (b if x > b else x)


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


def draw_metaballs(screen, particles, width, height):
    """
    Draw particles using metaball rendering for smooth liquid visuals
    """
    # Low-res buffer
    w2, h2 = width // RENDER_SCALE, height // RENDER_SCALE
    field = np.zeros((h2, w2), dtype=np.float32)

    # Precompute a grid of pixel coordinates (low-res)
    ys = np.arange(h2, dtype=np.float32)[:, None]
    xs = np.arange(w2, dtype=np.float32)[None, :]

    # Add each particle's contribution to the field
    # field += strength / (r^2 + soften)
    for p in particles:
        x, y = p["body"].position
        cx = (x / RENDER_SCALE)
        cy = (y / RENDER_SCALE)

        dx = xs - cx
        dy = ys - cy
        field += FIELD_STRENGTH / (dx*dx + dy*dy + FIELD_SOFTEN)

    # Threshold to get silhouette
    mask = field >= ISO_THRESHOLD

    # Create an RGB image (low-res), then scale up
    img = np.zeros((h2, w2, 3), dtype=np.uint8)
    img[mask] = (120, 190, 255)  # Liquid color

    surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))  # pygame expects (w,h,3)
    surf = pygame.transform.smoothscale(surf, (width, height))
    screen.blit(surf, (0, 0))


# ----------------------------
# Emitter Class
# ----------------------------
class Emitter:
    """
    Emits particles at a position, with a direction/spread and initial speed.
    Emits a new particle every 0.4-1.4 seconds (random) after the previous emission.
    """
    def __init__(self, name, get_position_func, direction=(0, -1), spread_deg=EMISSION_SPREAD_DEG, speed=PARTICLE_SPEED):
        """
        Initialize emitter
        
        Args:
            name: Unique name for this emitter
            get_position_func: Function that returns (x, y) position for emission point
            direction: Normalized direction vector (default: upward)
            spread_deg: Cone spread in degrees
            speed: Initial speed of particles
        """
        self.name = name
        self.get_position_func = get_position_func
        dx, dy = direction
        mag = math.hypot(dx, dy) or 1.0
        self.dir = (dx / mag, dy / mag)
        self.spread = math.radians(spread_deg)
        self.speed = speed
        
        # Timing: random interval between emissions
        self.time_until_next_emit = random.uniform(MIN_EMIT_INTERVAL, MAX_EMIT_INTERVAL)
        self.last_emit_time = 0.0

    def update(self, dt):
        """
        Update emitter timing
        Returns True if a particle should be emitted, False otherwise
        """
        self.time_until_next_emit -= dt
        if self.time_until_next_emit <= 0.0:
            # Time to emit! Schedule next emission with random interval
            self.time_until_next_emit = random.uniform(MIN_EMIT_INTERVAL, MAX_EMIT_INTERVAL)
            return True
        return False

    def get_position(self):
        """Get current emission position"""
        return self.get_position_func()

    def sample_velocity(self):
        """Sample random velocity within cone spread"""
        # Random cone around direction
        angle = (random.random() - 0.5) * self.spread
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = self.dir
        # Rotate (dx, dy) by angle
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        sp = self.speed * (0.75 + 0.5 * random.random())
        return (rx * sp, ry * sp)


# ----------------------------
# Fluid Emitter System
# ----------------------------
class FluidEmitterSystem:
    """
    Manages fluid particle emitters attached to stick figure body parts.
    Creates multiple emitters per limb segment and many emitters in body and head.
    """
    
    def __init__(self, stick_figure, space, screen_width, screen_height):
        """
        Initialize fluid emitter system
        
        Args:
            stick_figure: StickFigure instance
            space: pymunk.Space instance
            screen_width: Screen width in pixels
            screen_height: Screen height in pixels
        """
        self.stick_figure = stick_figure
        self.space = space
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # Particle storage: list of dict {body, shape, age}
        self.particles = []
        self.max_particles = 2200  # Cap for performance
        
        # Create emitters for all body parts
        self.emitters = []
        self._create_emitters()
        
        # Screen boundaries for collision
        self._create_screen_boundaries()
        
        # Prevent particles from colliding with stick figure
        # Approach: Modify stick figure shapes' collision masks to exclude particle category (0x8000)
        # This ensures stick figure bodies won't collide with particles
        PARTICLE_CATEGORY = 0x8000
        
        # Modify all stick figure shapes' collision masks to exclude particles
        for body_name, body in self.stick_figure.bodies.items():
            if body_name == 'center_pivot':  # Skip static pivot
                continue
            # Get all shapes attached to this body
            for shape in body.shapes:
                if hasattr(shape, 'filter') and shape.filter:
                    # Exclude particle category from mask
                    current_mask = shape.filter.mask
                    new_mask = current_mask & ~PARTICLE_CATEGORY
                    if new_mask != current_mask:  # Only update if changed
                        shape.filter = pymunk.ShapeFilter(
                            categories=shape.filter.categories,
                            mask=new_mask,
                            group=shape.filter.group
                        )
        
        print(f"FluidEmitterSystem initialized with {len(self.emitters)} emitters")

    def _create_screen_boundaries(self):
        """Create static boundaries at screen edges for particle collision"""
        static = self.space.static_body
        margin = 30
        segs = [
            pymunk.Segment(static, (margin, margin), (self.screen_width - margin, margin), 6),  # Top
            pymunk.Segment(static, (margin, self.screen_height - margin), 
                          (self.screen_width - margin, self.screen_height - margin), 6),  # Bottom
            pymunk.Segment(static, (margin, margin), (margin, self.screen_height - margin), 6),  # Left
            pymunk.Segment(static, (self.screen_width - margin, margin), 
                          (self.screen_width - margin, self.screen_height - margin), 6),  # Right
        ]
        for s in segs:
            s.friction = 0.7
            s.elasticity = 0.05
            s.collision_type = 2  # Boundaries
            # Boundaries collide with particles (category 0x8000)
            # Boundary category: 0x4000, mask: 0x8000 (can collide with particles)
            s.filter = pymunk.ShapeFilter(
                categories=0x4000,  # Boundary category
                mask=0x8000  # Collide with particles (category 0x8000)
            )
        self.space.add(*segs)
        self.boundaries = segs

    def _get_body_position(self, body_name):
        """Helper to get position of a body part"""
        def get_pos():
            body = self.stick_figure.bodies.get(body_name)
            if body is None:
                return (0, 0)
            pos = body.position
            return (float(pos.x) if hasattr(pos, 'x') else float(pos[0]),
                    float(pos.y) if hasattr(pos, 'y') else float(pos[1]))
        return get_pos

    def _get_limb_segment_emitters(self, body_name, count=3):
        """
        Create multiple emitters along a limb segment
        
        Args:
            body_name: Name of body part in stick_figure.bodies
            count: Number of emitters to create along the segment
        """
        emitters = []
        body = self.stick_figure.bodies.get(body_name)
        if body is None:
            return emitters
        
        # Get limb length if available
        length = self.stick_figure.limb_lengths.get(body_name, 40.0)
        
        # Create emitters distributed along the limb segment
        for i in range(count):
            t = (i + 1) / (count + 1)  # Position along segment (0 to 1)
            
            # Calculate position along limb segment
            # Limb segments are oriented along their body angle
            def get_pos_at_t(t_val=t, body_name=body_name, length=length):
                body = self.stick_figure.bodies.get(body_name)
                if body is None:
                    return (0, 0)
                # Position along segment: from -length/2 to +length/2
                offset = (t_val - 0.5) * length
                angle = body.angle
                local_x = offset * math.cos(angle)
                local_y = offset * math.sin(angle)
                
                # Transform to world space
                world_pos = body.local_to_world((local_x, local_y))
                return (float(world_pos.x) if hasattr(world_pos, 'x') else float(world_pos[0]),
                        float(world_pos.y) if hasattr(world_pos, 'y') else float(world_pos[1]))
            
            # Emit outward from limb (upward direction, will be adjusted by spread)
            # Direction is relative to world, not limb orientation
            direction = (0, -1)  # Upward by default
            
            emitter = Emitter(
                name=f"{body_name}_emitter_{i}",
                get_position_func=get_pos_at_t,
                direction=direction,
                spread_deg=EMISSION_SPREAD_DEG,
                speed=PARTICLE_SPEED
            )
            emitters.append(emitter)
        
        return emitters

    def _get_body_emitters(self, body_name, count=5):
        """
        Create multiple emitters on body parts (head, torso)
        
        Args:
            body_name: Name of body part ('head' or 'torso')
            count: Number of emitters to create
        """
        emitters = []
        body = self.stick_figure.bodies.get(body_name)
        if body is None:
            return emitters
        
        if body_name == 'head':
            # For head (circle), distribute emitters around circumference
            radius = self.stick_figure.head_radius * self.stick_figure.scale
            for i in range(count):
                angle = (2 * math.pi * i) / count
                def get_pos_at_angle(angle_val=angle, body_name=body_name, radius=radius):
                    body = self.stick_figure.bodies.get(body_name)
                    if body is None:
                        return (0, 0)
                    # Calculate local offset from center (in local coordinates)
                    local_offset_x = radius * math.cos(angle_val)
                    local_offset_y = radius * math.sin(angle_val)
                    # Transform to world space (accounting for body rotation)
                    world_pos = body.local_to_world((local_offset_x, local_offset_y))
                    return (float(world_pos.x) if hasattr(world_pos, 'x') else float(world_pos[0]),
                            float(world_pos.y) if hasattr(world_pos, 'y') else float(world_pos[1]))
                
                # Emit outward from center (direction relative to world, fixed for simplicity)
                # Position will update as body rotates, which is most important
                direction = (math.cos(angle), math.sin(angle))
                emitter = Emitter(
                    name=f"{body_name}_emitter_{i}",
                    get_position_func=get_pos_at_angle,
                    direction=direction,
                    spread_deg=EMISSION_SPREAD_DEG,
                    speed=PARTICLE_SPEED
                )
                emitters.append(emitter)
                
        elif body_name == 'torso':
            # For torso (triangle), distribute emitters along edges
            # Torso is an inverted triangle: wide at top, point at bottom
            torso_length = self.stick_figure.torso_length * self.stick_figure.scale
            shoulder_width = (self.stick_figure.shoulder_width / 2) * self.stick_figure.scale
            
            for i in range(count):
                # Distribute along triangle perimeter
                # Top edge (shoulders), sides, or bottom point
                t = i / (count - 1) if count > 1 else 0.5
                
                def get_pos_at_t_torso(t_val=t, body_name=body_name, torso_length=torso_length, shoulder_width=shoulder_width):
                    body = self.stick_figure.bodies.get(body_name)
                    if body is None:
                        return (0, 0)
                    pos = body.position
                    center_x = float(pos.x) if hasattr(pos, 'x') else float(pos[0])
                    center_y = float(pos.y) if hasattr(pos, 'y') else float(pos[1])
                    
                    # Calculate position on triangle based on t
                    # t=0: left shoulder, t=0.33: top center, t=0.66: right shoulder, t=1: bottom point
                    if t_val < 0.33:
                        # Left shoulder to top center
                        local_x = -shoulder_width + (t_val / 0.33) * shoulder_width
                        local_y = -torso_length / 2
                    elif t_val < 0.66:
                        # Top center to right shoulder
                        local_x = (t_val - 0.33) / 0.33 * shoulder_width
                        local_y = -torso_length / 2
                    else:
                        # Right shoulder to bottom point (along right edge)
                        t_edge = (t_val - 0.66) / 0.34
                        local_x = shoulder_width * (1 - t_edge)
                        local_y = -torso_length / 2 + t_edge * torso_length
                    
                    # Transform to world space
                    angle = body.angle
                    cos_a = math.cos(angle)
                    sin_a = math.sin(angle)
                    world_x = center_x + local_x * cos_a - local_y * sin_a
                    world_y = center_y + local_x * sin_a + local_y * cos_a
                    return (world_x, world_y)
                
                # Emit outward perpendicular to surface
                # For simplicity, emit upward/outward
                direction = (0, -1)
                emitter = Emitter(
                    name=f"{body_name}_emitter_{i}",
                    get_position_func=get_pos_at_t_torso,
                    direction=direction,
                    spread_deg=EMISSION_SPREAD_DEG,
                    speed=PARTICLE_SPEED
                )
                emitters.append(emitter)
        
        return emitters

    def _create_emitters(self):
        """Create emitters for all body parts"""
        # Head: many emitters (5)
        self.emitters.extend(self._get_body_emitters('head', count=5))
        
        # Torso: many emitters (5)
        self.emitters.extend(self._get_body_emitters('torso', count=5))
        
        # Each limb segment: multiple emitters (3 per segment)
        limb_segments = [
            'left_upper_arm', 'left_forearm',
            'right_upper_arm', 'right_forearm',
            'left_thigh', 'left_shin',
            'right_thigh', 'right_shin'
        ]
        
        for limb in limb_segments:
            self.emitters.extend(self._get_limb_segment_emitters(limb, count=3))

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
        shape.collision_type = 1  # Particles
        
        # Set collision category for particles (don't collide with stick figure)
        # Use a unique category bit that stick figure bodies don't have in their mask
        # Particles category: 0x8000 (high bit not used by stick figure)
        # Particles mask: 0x8000 | 0x4000 (collide with other particles and boundaries)
        shape.filter = pymunk.ShapeFilter(
            categories=0x8000,  # Particle category
            mask=0x8000 | 0x4000  # Collide with other particles (0x8000) and boundaries (0x4000)
        )
        
        self.space.add(body, shape)
        return body, shape

    def update(self, dt):
        """Update emitters and particles"""
        dt = clamp(dt, 0.0, 1.0 / 20.0)
        
        # Check each emitter for emission
        for emitter in self.emitters:
            if emitter.update(dt):
                # Time to emit!
                if len(self.particles) >= self.max_particles:
                    continue
                
                # Get position and velocity
                try:
                    pos = emitter.get_position()
                    vx, vy = emitter.sample_velocity()
                    px, py = pos
                    # Add small jitter
                    px += random.uniform(-3, 3)
                    py += random.uniform(-3, 3)
                    
                    body, shape = self._make_particle(px, py, vx, vy)
                    self.particles.append({"body": body, "shape": shape, "age": 0.0})
                except Exception as e:
                    print(f"Error emitting particle from {emitter.name}: {e}")
                    continue
        
        # Build spatial hash for neighbor interactions
        positions = [p["body"].position for p in self.particles]
        grid = spatial_hash(positions)
        
        # Apply fluid forces (pressure + viscosity + cohesion)
        for i, p in enumerate(self.particles):
            bi = p["body"]
            xi, yi = bi.position
            vix, viy = bi.velocity
            
            cxi = int(xi // CELL)
            cyi = int(yi // CELL)
            
            fx = 0.0
            fy = 0.0
            
            for cell in neighbor_cells(cxi, cyi):
                for j in grid.get(cell, []):
                    if j == i:
                        continue
                    bj = self.particles[j]["body"]
                    xj, yj = bj.position
                    dx = xi - xj
                    dy = yi - yj
                    r2 = dx * dx + dy * dy
                    if r2 <= 1e-9:
                        continue
                    if r2 > NEIGHBOR_RADIUS * NEIGHBOR_RADIUS:
                        continue
                    
                    r = math.sqrt(r2)
                    
                    # Cohesion: pull particles together at mid-range
                    if r > REST_DIST * COHESION_START and r < REST_DIST * COHESION_END:
                        nx = dx / r
                        ny = dy / r
                        t = (r - REST_DIST * COHESION_START) / (REST_DIST * (COHESION_END - COHESION_START))
                        pull = (1.0 - abs(2.0 * t - 1.0))  # Triangle profile
                        strength = pull * COHESION
                        fx -= nx * strength
                        fy -= ny * strength
                    
                    # Pressure: push away if too close
                    overlap = (REST_DIST - r)
                    if overlap > 0:
                        nx = dx / r
                        ny = dy / r
                        strength = (overlap / REST_DIST) * PRESSURE
                        fx += nx * strength
                        fy += ny * strength
                    
                    # Viscosity: damp relative velocities
                    vjx, vjy = bj.velocity
                    rvx = vjx - vix
                    rvy = vjy - viy
                    fx += rvx * VISCOSITY
                    fy += rvy * VISCOSITY
            
            # Apply force
            bi.apply_force_at_local_point((fx, fy), (0, 0))
            
            # Apply gravity manually (since stick figure overrides space gravity)
            # Gravity force = mass * gravity_vector
            gravity_x, gravity_y = GRAVITY
            bi.apply_force_at_local_point((gravity_x * PARTICLE_MASS, gravity_y * PARTICLE_MASS), (0, 0))
        
        # Apply global damping
        dead = []
        for idx, p in enumerate(self.particles):
            p["age"] += dt
            b = p["body"]
            b.velocity = (b.velocity[0] * DAMPING, b.velocity[1] * DAMPING)
            
            # Remove particles that are too old or outside screen
            if p["age"] > 10.0:  # 10 second lifetime
                dead.append(idx)
                continue
            
            # Remove if far outside screen
            x, y = b.position
            margin = 200
            if x < -margin or x > self.screen_width + margin or \
               y < -margin or y > self.screen_height + margin:
                dead.append(idx)
        
        # Remove dead particles
        for idx in reversed(dead):
            self.space.remove(self.particles[idx]["shape"], self.particles[idx]["body"])
            self.particles.pop(idx)

    def draw(self, screen):
        """Draw particles using metaball rendering"""
        if len(self.particles) > 0:
            draw_metaballs(screen, self.particles, self.screen_width, self.screen_height)
