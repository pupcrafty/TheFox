# Corridor Scrolling Effect - 3D Perspective

## Overview

The corridor scrolling effect creates the illusion that the emitters are moving forward through an endless 3D corridor. Everything appears to recede into the distance using proper perspective projection along the depth (Z) axis. Particles and hallway segments get smaller as they move farther away, creating a convincing sense of depth and forward motion.

## Key Concepts

### 1. 3D Perspective Projection
- The system uses one-point perspective projection
- Objects have X, Y (2D screen position) and Z (depth) coordinates
- Z increases as objects move backward (away from camera)
- Objects are projected to 2D screen coordinates using perspective math
- Scale factor: `scale = FOV / (FOV + z)` - objects get smaller as Z increases

### 2. Fixed Emitters
- The four corner emitters remain fixed at their screen positions (Z = 0, near plane)
- They appear to be "moving forward" through the corridor
- Particles are emitted from these fixed positions with initial Z = NEAR_PLANE_Z

### 3. Scrolling in Z-Space
- All particles move backward in Z-space at a constant speed
- Z-depth increases over time: `z_depth += CORRIDOR_SCROLL_SPEED * dt`
- This creates the illusion that the emitters are moving forward
- Particles have greatly extended lifetimes (60 seconds) to allow them to travel far into the distance

### 4. Perspective-Scaled Wireframe Hallway
- Rectangular wireframe segments represent the corridor walls
- Segments exist in Z-space and scroll backward continuously
- Each segment is rendered with perspective projection - gets smaller as Z increases
- New segments are generated at the near plane as old ones scroll beyond the far plane
- The wireframe provides visual context for the 3D corridor concept

## Implementation Details

### Configuration Constants

```python
CORRIDOR_SCROLL_SPEED = 200.0  # Units per second - how fast everything moves backward in Z
PARTICLE_LIFETIME = 60.0  # Seconds - greatly extended lifetime
HALLWAY_SEGMENT_LENGTH = 400  # Length of each hallway segment in Z-space
PERSPECTIVE_FOV = 800.0  # Field of view for perspective projection (larger = wider view)
VANISHING_POINT_Y = 0.5  # Y position of vanishing point (0.0 = top, 1.0 = bottom, 0.5 = center)
NEAR_PLANE_Z = 100.0  # Z distance of near plane (closest visible objects)
FAR_PLANE_Z = 5000.0  # Z distance of far plane (furthest visible objects)
```

### Perspective Projection

The core of the 3D effect is the perspective projection function:

```python
def project_3d_to_2d(x, y, z, screen_width, screen_height):
    """
    Project 3D point (x, y, z) to 2D screen coordinates using perspective projection
    Returns: (screen_x, screen_y, scale_factor)
    """
    scale = PERSPECTIVE_FOV / (PERSPECTIVE_FOV + z)
    vanishing_x = screen_width / 2
    vanishing_y = screen_height * VANISHING_POINT_Y
    screen_x = vanishing_x + (x - vanishing_x) * scale
    screen_y = vanishing_y + (y - vanishing_y) * scale
    return screen_x, screen_y, scale
```

**Key Points:**
- Scale factor decreases as Z increases (objects get smaller)
- Vanishing point is at screen center horizontally, configurable vertically
- Objects converge toward the vanishing point as they recede

### Particle System with Z-Depth

1. **Particle Creation**
   - Particles are created with `z_depth = NEAR_PLANE_Z`
   - They maintain their 2D physics (X, Y) while Z increases over time
   - Each particle stores: `{"z_depth": z_value, ...}`

2. **Scrolling Mechanism**
   - Each frame: `particle["z_depth"] += CORRIDOR_SCROLL_SPEED * dt`
   - Particles move backward in Z-space (not Y-space)
   - 2D physics (gravity, fluid forces) still apply to X, Y coordinates

3. **Rendering with Perspective**
   - Particles are projected to screen using `project_3d_to_2d()`
   - Particle size scales with depth: `scaled_radius = PARTICLE_RADIUS * scale`
   - Field strength also scales: `depth_field_strength = FIELD_STRENGTH * scale`
   - This makes distant particles smaller and less intense

4. **Particle Removal**
   - Particles removed when `z_depth > FAR_PLANE_Z`
   - Also removed if outside 2D screen bounds
   - Extended lifetime (60 seconds) allows particles to travel far

### Hallway Wireframe with Perspective

1. **Segment Structure**
   - Each segment has: `z_depth`, `base_width`, `base_height`, `segment_length`
   - Segments exist in Z-space, not Y-space
   - Base dimensions are at the near plane

2. **Scrolling**
   - Segments scroll backward: `segment["z_depth"] += scroll_delta_z`
   - Segments beyond `FAR_PLANE_Z` are removed
   - New segments spawn at `NEAR_PLANE_Z - segment_length`

3. **Perspective Rendering**
   - Each segment is projected using perspective
   - Width and height scale: `scaled_width = base_width * scale`
   - Rectangle is centered horizontally, positioned vertically at vanishing point
   - Creates the converging tunnel effect

4. **Visual Effect**
   - Segments get progressively smaller as they recede
   - Creates the illusion of an endless corridor
   - Wireframe clearly shows the 3D perspective concept

## Visual Effect

The combined effect creates:

1. **Forward Motion Illusion**
   - Fixed emitters appear to move forward
   - Particles stream backward, getting smaller with distance
   - Hallway segments scroll backward, converging to vanishing point

2. **Depth Perception**
   - Perspective projection creates realistic depth
   - Objects scale correctly with distance
   - Vanishing point creates sense of infinite corridor

3. **Particle Behavior**
   - Particles maintain 2D physics (gravity, fluid forces)
   - They scroll backward in Z while interacting in X, Y
   - Size and intensity decrease with distance
   - Extended lifetime allows complex interactions

4. **Wireframe Visualization**
   - Clearly shows the corridor structure
   - Demonstrates perspective convergence
   - Makes the 3D concept visible and understandable

## Technical Considerations

### Coordinate System
- **Screen coordinates**: (0,0) at top-left, X increases right, Y increases down
- **3D coordinates**: X, Y (2D position), Z (depth, increases backward)
- **Projection**: 3D → 2D using perspective math
- **Vanishing point**: Center horizontally, configurable vertically

### Performance
- Perspective calculations are simple (just scale factor)
- Segment culling prevents drawing off-screen segments
- Particle limit (2200) prevents performance issues
- Efficient wireframe rendering (4 lines per segment)

### Boundary Handling
- Particles can have large Z values (up to FAR_PLANE_Z)
- 2D boundaries still apply to X, Y coordinates
- Z-based removal prevents infinite accumulation

### Perspective Math
- Scale: `FOV / (FOV + z)` - simple and efficient
- Larger FOV = wider view angle
- Objects at z=0 have scale=1.0 (full size)
- Objects at z=FOV have scale=0.5 (half size)
- Objects at z=∞ have scale→0 (invisible)

## Code Locations

- **Configuration**: Lines 60-67 in `app.py`
- **Perspective functions**: `project_3d_to_2d()`, `get_perspective_scale()` after `clamp()`
- **Scroll offset**: `self.scroll_offset_z` in `__init__`
- **Particle Z-depth**: Added in particle creation (line ~1307)
- **Z scrolling**: After physics step in `update()` method
- **Perspective rendering**: `draw_metaballs()` function - projects particles
- **Hallway segments**: `_initialize_hallway_segments()`, `_update_hallway_segments()`, `_draw_hallway_wireframe()`
- **Particle lifetime**: Line ~1411 (changed from 10.0 to PARTICLE_LIFETIME)

## Testing

To test the effect:

1. Run the application
2. Observe particles streaming backward, getting smaller with distance
3. Verify wireframe hallway segments scrolling and converging to vanishing point
4. Check that new segments appear at near plane
5. Confirm particles have extended lifetime (60 seconds)
6. Verify emitters remain fixed at screen positions
7. Observe perspective scaling - distant objects are smaller

## Visual Reference

The effect should match a one-point perspective corridor:
- Rectangular frames that get smaller as they recede
- All lines converge to a central vanishing point
- Particles distributed throughout 3D space
- Particles scale with distance (larger in foreground, smaller in background)
- Creates convincing illusion of forward motion through endless tunnel

## Future Enhancements

Potential improvements:

1. **Enhanced Perspective**
   - Add floor/ceiling grid lines for stronger depth cues
   - Vary opacity based on distance (fog effect)
   - Add texture or pattern to hallway segments

2. **Particle Effects**
   - Motion blur for fast-moving particles
   - Depth-based opacity (fade with distance)
   - Particle trails for speed visualization

3. **Dynamic Effects**
   - Variable scroll speed based on sensor input
   - Acceleration/deceleration effects
   - Curved corridor paths

4. **Visual Polish**
   - Lighting effects based on depth
   - Color gradients (warmer near, cooler far)
   - Particle glow effects

## Notes

- The effect works best with active particle emission
- Sensor input controls emission rate and particle velocity
- The wireframe makes the 3D corridor concept clearly visible
- Extended particle lifetime allows for more complex fluid interactions
- Perspective projection creates realistic depth perception
- All objects properly scale with distance for convincing 3D effect
