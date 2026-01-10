# FLUIDS_GUIDE.md
### 2D Particle Fluids in Pygame + Pymunk

This module implements **real-time 2D liquid simulation** using:
- **Pymunk** for physics (particles + collisions)
- **Pygame** for rendering
- **Particle-based fluid approximation** (pressure, viscosity, cohesion)
- **Metaballs / iso-surface rendering** for smooth liquid visuals
- **Multiple continuous emitters**

This is **not full SPH**, but a performant hybrid suitable for games, art tools, and interactive systems.

---

## 1. Core Concepts

### 1.1 Representation
- Liquid is represented as **many small circular particles**
- Each particle has:
  - Mass
  - Radius
  - Lifetime
  - Velocity
- Fluid behavior emerges from **local forces between nearby particles**

### 1.2 Forces Used
| Force        | Purpose |
|-------------|--------|
| Gravity     | Pulls liquid downward |
| Pressure    | Prevents particles from overlapping |
| Viscosity   | Damps relative motion (gooey vs watery) |
| Cohesion    | Surface tension (keeps liquid together) |
| Damping     | Global energy loss (stability) |

---

## 2. Emitters (Sources)

Emitters continuously spawn particles.

### Key Controls
```python
PARTICLES_PER_SEC_PER_EMITTER = 120
MAX_PARTICLES = 1400
PARTICLE_LIFETIME = 6.0
```

---

## 3. Particle Decay

Particles are removed from the simulation through two mechanisms:

### 3.1 Time-Based Decay
Particles are removed after a fixed lifetime regardless of position:
```python
PARTICLE_LIFETIME = 9.0  # seconds
```

### 3.2 Distance-Based Decay
Particles that collect into blobs far from emitters gradually lose mass/particles:

```python
DECAY_START_DISTANCE = 400.0  # distance from nearest emitter where decay begins
DECAY_MAX_DISTANCE = 600.0    # distance where decay probability reaches 100%
DECAY_RATE = 0.02             # probability per frame at max distance
```

**How it works:**
- Particles within `DECAY_START_DISTANCE` of any emitter: **no decay**
- Between `DECAY_START_DISTANCE` and `DECAY_MAX_DISTANCE`: **gradual decay** (linear probability scaling)
- At or beyond `DECAY_MAX_DISTANCE`: **maximum decay rate** applied
- Decay probability scales linearly: `0.0` at start distance → `DECAY_RATE` at max distance

This prevents collected blobs from accumulating indefinitely while allowing particles close to emitters to remain stable.
