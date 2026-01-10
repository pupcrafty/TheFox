import math
import random
import numpy as np
import pygame
import pymunk
from collections import defaultdict

# ----------------------------
# Config
# ----------------------------
W, H = 1000, 650
FPS = 60

GRAVITY = (0, 1200)
PARTICLE_RADIUS = 4
PARTICLE_MASS = 0.25
PARTICLE_FRICTION = 0.2
PARTICLE_ELASTICITY = 0.0

MAX_PARTICLES = 2200          # cap for performance
PARTICLES_PER_SEC_PER_EMITTER = 10
PARTICLE_LIFETIME = 9.0       # seconds

# Distance-based decay: particles far from emitters lose mass/particles
DECAY_START_DISTANCE = 400.0  # distance from nearest emitter where decay begins
DECAY_MAX_DISTANCE = 600.0    # distance where decay probability reaches 100%
DECAY_RATE = 0.02             # probability per frame at max distance (0-1 scale with distance)

# "Fluid-ish" forces
VISCOSITY = 0.10              # higher => gooey
PRESSURE = 1800.0             # higher => stiffer / more incompressible
REST_DIST = PARTICLE_RADIUS * 2.2  # preferred spacing
NEIGHBOR_RADIUS = REST_DIST * 2.2  # interaction distance
DAMPING = 0.995               # global velocity damping per step

# Spatial hash cell size (close to neighbor radius)
CELL = int(NEIGHBOR_RADIUS)

RENDER_SCALE = 4             # render at 1/4 resolution then scale up (speed)
ISO_THRESHOLD = 1.15         # higher => thinner liquid, lower => fatter
FIELD_STRENGTH = 70.0        # higher => blobs merge more
FIELD_SOFTEN = 3.0           # avoids singularities near particle centers

COHESION = 260.0      # surface tension-ish pull (higher => stickier)
COHESION_START = 0.9  # start pulling at ~0.9 * REST_DIST
COHESION_END = 2.0    # stop pulling at ~2.0 * REST_DIST

# ----------------------------
# Helpers
# ----------------------------
def clamp(x, a, b):
    return a if x < a else (b if x > b else x)

def draw_metaballs(screen, particles):
    # Low-res buffer
    w2, h2 = W // RENDER_SCALE, H // RENDER_SCALE
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
    img[mask] = (120, 190, 255)  # liquid color

    surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))  # pygame expects (w,h,3)
    surf = pygame.transform.smoothscale(surf, (W, H))
    screen.blit(surf, (0, 0))

class Emitter:
    """
    Emits particles at a position, with a direction/spread and initial speed.
    """
    def __init__(self, pos, direction=(0, -1), spread_deg=18, speed=520):
        self.x, self.y = pos
        dx, dy = direction
        mag = math.hypot(dx, dy) or 1.0
        self.dir = (dx / mag, dy / mag)
        self.spread = math.radians(spread_deg)
        self.speed = speed
        self.accum = 0.0

    def update(self, dt, spawn_rate):
        self.accum += dt * spawn_rate
        n = int(self.accum)
        self.accum -= n
        return n

    def sample_velocity(self):
        # random cone around direction
        angle = (random.random() - 0.5) * self.spread
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = self.dir
        # rotate (dx, dy) by angle
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        sp = self.speed * (0.75 + 0.5 * random.random())
        return (rx * sp, ry * sp)

def add_static_boundaries(space):
    static = space.static_body
    segs = [
        pymunk.Segment(static, (30, 30), (W - 30, 30), 6),
        pymunk.Segment(static, (30, H - 30), (W - 30, H - 30), 6),
        pymunk.Segment(static, (30, 30), (30, H - 30), 6),
        pymunk.Segment(static, (W - 30, 30), (W - 30, H - 30), 6),
    ]
    for s in segs:
        s.friction = 0.7
        s.elasticity = 0.05
    space.add(*segs)

def make_particle(space, x, y, vx, vy):
    body = pymunk.Body()
    body.position = (x, y)
    body.velocity = (vx, vy)
    body.mass = PARTICLE_MASS
    # moment for circle
    body.moment = pymunk.moment_for_circle(PARTICLE_MASS, 0, PARTICLE_RADIUS)
    shape = pymunk.Circle(body, PARTICLE_RADIUS)
    shape.friction = PARTICLE_FRICTION
    shape.elasticity = PARTICLE_ELASTICITY
    shape.collision_type = 1
    space.add(body, shape)
    return body, shape

def spatial_hash(positions):
    grid = defaultdict(list)
    for i, (x, y) in enumerate(positions):
        cx = int(x // CELL)
        cy = int(y // CELL)
        grid[(cx, cy)].append(i)
    return grid

def neighbor_cells(cx, cy):
    for ox in (-1, 0, 1):
        for oy in (-1, 0, 1):
            yield (cx + ox, cy + oy)

def distance_to_nearest_emitter(particle_pos, emitters):
    """Calculate the distance from particle position to the nearest emitter."""
    px, py = particle_pos
    min_dist = float('inf')
    for em in emitters:
        dx = px - em.x
        dy = py - em.y
        dist = math.hypot(dx, dy)
        min_dist = min(min_dist, dist)
    return min_dist

def should_decay_by_distance(dist):
    """Returns probability of decay (0.0 to 1.0) based on distance from emitter."""
    if dist < DECAY_START_DISTANCE:
        return 0.0  # No decay if close enough
    if dist >= DECAY_MAX_DISTANCE:
        return DECAY_RATE  # Full decay rate at max distance
    
    # Linear interpolation between start and max distance
    t = (dist - DECAY_START_DISTANCE) / (DECAY_MAX_DISTANCE - DECAY_START_DISTANCE)
    return DECAY_RATE * t

# ----------------------------
# Main
# ----------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()
    pygame.display.set_caption("Pygame + Pymunk: pumped 2D particle fluid")

    space = pymunk.Space()
    space.gravity = GRAVITY
    space.iterations = 12  # collision solver iterations
    add_static_boundaries(space)

    emitters = [
        Emitter((W * 0.25, H * 0.85), direction=(0.2, -1), spread_deg=22, speed=620),
        Emitter((W * 0.50, H * 0.85), direction=(0.0, -1), spread_deg=18, speed=680),
        Emitter((W * 0.75, H * 0.85), direction=(-0.2, -1), spread_deg=22, speed=620),
    ]

    particles = []  # list of dict {body, shape, age}
    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0
        dt = clamp(dt, 0.0, 1.0 / 20.0)

        # Events
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False

        # Spawn new particles
        spawn_rate = PARTICLES_PER_SEC_PER_EMITTER
        for em in emitters:
            n = em.update(dt, spawn_rate)
            for _ in range(n):
                if len(particles) >= MAX_PARTICLES:
                    break
                vx, vy = em.sample_velocity()
                # small jitter so they don't stack perfectly
                px = em.x + random.uniform(-3, 3)
                py = em.y + random.uniform(-3, 3)
                body, shape = make_particle(space, px, py, vx, vy)
                particles.append({"body": body, "shape": shape, "age": 0.0})

        # Build spatial hash for neighbor interactions
        positions = [p["body"].position for p in particles]
        grid = spatial_hash(positions)

        # Apply "fluid" forces (pressure + viscosity) using neighbor list
        # NOTE: This is not full SPH, but it gives a convincing liquid-y push.
        for i, p in enumerate(particles):
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
                    bj = particles[j]["body"]
                    xj, yj = bj.position
                    dx = xi - xj
                    dy = yi - yj
                    r2 = dx * dx + dy * dy
                    if r2 <= 1e-9:
                        continue
                    if r2 > NEIGHBOR_RADIUS * NEIGHBOR_RADIUS:
                        continue

                    r = math.sqrt(r2)

                    if r > REST_DIST * COHESION_START and r < REST_DIST * COHESION_END:
                        nx = dx / r
                        ny = dy / r
                        # Pull strength peaks mid-range and fades at ends (simple bell-ish curve)
                        t = (r - REST_DIST * COHESION_START) / (REST_DIST * (COHESION_END - COHESION_START))
                        pull = (1.0 - abs(2.0 * t - 1.0))  # triangle profile 0..1..0
                        strength = pull * COHESION
                        fx -= nx * strength
                        fy -= ny * strength
                    # pressure: push away if closer than rest distance
                    overlap = (REST_DIST - r)
                    if overlap > 0:
                        nx = dx / r
                        ny = dy / r
                        strength = (overlap / REST_DIST) * PRESSURE
                        fx += nx * strength
                        fy += ny * strength

                    # viscosity: damp relative velocities
                    vjx, vjy = bj.velocity
                    rvx = vjx - vix
                    rvy = vjy - viy
                    fx += rvx * VISCOSITY
                    fy += rvy * VISCOSITY

            # apply force (scaled by dt so it’s stable)
            bi.apply_force_at_local_point((fx, fy), (0, 0))

        # Step physics
        space.step(dt)

        # Global damping + lifetime and distance-based decay cleanup
        dead = []
        for idx, p in enumerate(particles):
            p["age"] += dt
            b = p["body"]
            b.velocity = (b.velocity[0] * DAMPING, b.velocity[1] * DAMPING)
            
            # Time-based decay: remove if lifetime exceeded
            if p["age"] > PARTICLE_LIFETIME:
                dead.append(idx)
                continue
            
            # Distance-based decay: particles far from emitters lose mass/particles
            dist = distance_to_nearest_emitter(b.position, emitters)
            decay_prob = should_decay_by_distance(dist)
            if decay_prob > 0.0 and random.random() < decay_prob:
                dead.append(idx)

        # Remove oldest first (reverse indices)
        for idx in reversed(dead):
            space.remove(particles[idx]["shape"], particles[idx]["body"])
            particles.pop(idx)

        # Draw
        screen.fill((12, 12, 16))

        # boundaries (just draw the box)
        pygame.draw.rect(screen, (60, 60, 70), pygame.Rect(30, 30, W - 60, H - 60), 2)

        # emitters
        for em in emitters:
            pygame.draw.circle(screen, (240, 170, 60), (int(em.x), int(em.y)), 8)

        # particles
        draw_metaballs(screen, particles)
        # HUD
        font = pygame.font.SysFont(None, 22)
        txt = font.render(f"particles: {len(particles)}  (ESC to quit)", True, (220, 220, 220))
        screen.blit(txt, (40, 40))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()