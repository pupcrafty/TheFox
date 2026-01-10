# Stick Figure Physics Rewrite

## Goal
Completely rewrite the stick figure physics system with improved constraints, body rotation control, and limb management. The system should support input from peripheral devices (to be integrated later) and also support preloaded force sequences from JSON files for testing and demonstration.

## Key Requirements

### Input System
- Design input interface that can accept peripheral device inputs (to be implemented later)
- Support JSON file format for preloaded force sequences
- Forces apply for a moment then fade out naturally
- Forces can ONLY be applied to sensor points: `left_upper_arm`, `right_upper_arm`, `left_thigh`, `right_thigh`
- Torso is NOT a valid sensor target - body rotation/spin must be achieved through forces on sensor points

### Body Constraints
- Body (torso) should have a **maximum tilt angle** (e.g., ±45 degrees from vertical)
- Body should **never flip "ass up"** (i.e., torso should not rotate beyond ±90 degrees from upright)
- Body should be able to **spin 180 degrees in place around the vertical axis** while maintaining tilt constraints
- Display visual indicator on body showing "F" (front) or "B" (back) to indicate which side is facing the viewer

### Limb Constraints
- Limbs should **not cross each other** (collision detection/prevention)
- Limbs should **normalize to the correct side** of the body (left limbs stay on left, right on right)
- When limbs try to cross, apply corrective forces to push them back to their proper side

## Implementation Steps

### Step 1: JSON Force Definition Format
Design JSON structure for preloaded force sequences:

```json
{
  "sequences": [
    {
      "name": "jump",
      "forces": [
        {
          "time": 0.0,
          "target": "left_thigh",
          "force": [0, -500],
          "torque": 0,
          "duration": 0.2,
          "fade_type": "linear"
        },
        {
          "time": 0.0,
          "target": "right_thigh",
          "force": [0, -500],
          "torque": 0,
          "duration": 0.2,
          "fade_type": "linear"
        },
        {
          "time": 0.1,
          "target": "left_upper_arm",
          "force": [0, -200],
          "torque": 0,
          "duration": 0.15,
          "fade_type": "exponential"
        },
        {
          "time": 0.1,
          "target": "right_upper_arm",
          "force": [0, -200],
          "torque": 0,
          "duration": 0.15,
          "fade_type": "exponential"
        }
      ]
    },
    {
      "name": "spin",
      "forces": [
        {
          "time": 0.0,
          "target": "left_upper_arm",
          "force": [100, 0],
          "torque": 0,
          "duration": 0.5,
          "fade_type": "linear"
        },
        {
          "time": 0.0,
          "target": "right_upper_arm",
          "force": [-100, 0],
          "torque": 0,
          "duration": 0.5,
          "fade_type": "linear"
        },
        {
          "time": 0.0,
          "target": "left_thigh",
          "force": [80, 0],
          "torque": 0,
          "duration": 0.5,
          "fade_type": "linear"
        },
        {
          "time": 0.0,
          "target": "right_thigh",
          "force": [-80, 0],
          "torque": 0,
          "duration": 0.5,
          "fade_type": "linear"
        }
      ]
    }
  ]
}
```

**Force properties:**
- `time`: When to apply force (relative to sequence start, in seconds)
- `target`: Sensor point to apply force to - MUST be one of: `"left_upper_arm"`, `"right_upper_arm"`, `"left_thigh"`, `"right_thigh"`
- `force`: Linear force vector [fx, fy] in physics units
- `torque`: Rotational force (angular acceleration) in physics units (note: torque may not be used if sensor only provides linear forces)
- `duration`: How long force is active (seconds)
- `fade_type`: How force fades out - "linear", "exponential", or "instant"

### Step 2: Force Application System
Create `ForceSequence` class and `ForceManager`:

**ForceSequence class:**
- Load from JSON file
- Store sequence name and list of force definitions
- Track current time and active forces
- Apply forces to appropriate body parts
- Handle force fading (linear, exponential decay)

**ForceManager class:**
- Manage multiple force sequences
- Queue/play sequences
- Update active forces each frame
- Apply forces to stick figure bodies
- Remove expired forces

**Force structure:**
```python
{
    "body": pymunk.Body reference,  # One of the 4 sensor point bodies
    "force": (fx, fy),  # Linear force vector
    "torque": float,  # Rotational force (may be 0 if sensor only provides linear forces)
    "start_time": float,  # When force started (relative to sequence start)
    "duration": float,  # How long force lasts
    "fade_type": str,  # "linear", "exponential", or "instant"
    "elapsed": float  # Time elapsed since force started
}
```

### Step 3: Body Tilt Constraints
Implement maximum tilt and anti-flip constraints for torso:

**Tilt constraint:**
- Calculate current torso angle relative to vertical
- If tilt exceeds maximum (e.g., ±45°), apply restoring torque
- Restoring torque: `torque = -k * (angle - max_tilt)` where `k` is spring constant

**Anti-flip constraint:**
- Prevent torso from rotating beyond ±90° from upright
- Calculate dot product of torso "up" vector with world "up" vector
- If torso is pointing downward (flipped), apply strong restoring torque
- Use constraint: `if torso_angle < -90° or torso_angle > 90°: apply correction`

**Implementation approach:**
- Add constraint joints/motors (pymunk.RotaryLimitJoint or RotarySpring)
- Or apply corrective torques manually in update loop
- Use body local-to-world transform to determine torso orientation

### Step 4: Body 180-Degree Spin
Implement controlled rotation around vertical axis:

**Spin mechanism:**
- Allow torso to rotate 360° around vertical axis (world Z-axis)
- But maintain tilt constraints (body doesn't flip)
- Spin is achieved through forces on sensor points (upper arms and thighs)
- Apply opposing horizontal forces to left and right sensor points to create rotational moment
- Should be able to spin in place without affecting limb positions too much

**Implementation:**
- To spin clockwise: apply leftward force to right sensor points, rightward force to left sensor points
- To spin counter-clockwise: apply rightward force to right sensor points, leftward force to left sensor points
- Forces on upper arms and thighs work together to create net torque on torso
- Ensure spin doesn't violate tilt constraints
- Forces propagate through joints to create rotational motion

### Step 5: Limb Cross-Prevention
Implement system to prevent limbs from crossing:

**Left/Right side definition:**
- Define which side each limb belongs to based on body structure
- Left limbs: `left_upper_arm`, `left_forearm`, `left_thigh`, `left_shin`
- Right limbs: `right_upper_arm`, `right_forearm`, `right_thigh`, `right_shin`

**Cross-detection algorithm:**
- Each frame, check if any left limb has crossed to right side (or vice versa)
- Detection method: Project limb endpoints onto horizontal axis relative to torso
- If `left_limb_x > torso_x + threshold` or `right_limb_x < torso_x - threshold`, limb has crossed
- Also check limb-to-limb crossing (e.g., left arm crossing right arm)

**Correction forces:**
- When cross detected, apply force to push limb back to correct side
- Force direction: perpendicular to limb, toward correct side
- Force magnitude: proportional to how far limb has crossed
- Apply to limb's center of mass or appropriate joint

**Normalization:**
- When limbs are on correct side but close to midline, apply gentle corrective force
- Keeps limbs in their "comfort zone" on their respective sides
- Use soft constraints rather than hard limits for natural movement

### Step 6: Input Interface Design
Design input system for future peripheral integration:

**Input interface:**
```python
class StickFigureInput:
    """Interface for applying forces to stick figure"""
    
    # Valid sensor targets
    VALID_TARGETS = ["left_upper_arm", "right_upper_arm", "left_thigh", "right_thigh"]
    
    def apply_force(self, target: str, force: tuple, torque: float = 0.0):
        """Apply force to sensor point (one of the 4 valid targets)"""
        if target not in self.VALID_TARGETS:
            raise ValueError(f"Invalid target: {target}. Must be one of {self.VALID_TARGETS}")
        pass
    
    def apply_sequence(self, sequence_name: str):
        """Play a predefined force sequence"""
        pass
    
    def set_continuous_force(self, target: str, force: tuple, torque: float = 0.0):
        """Set continuous force (from peripheral device) - only valid sensor targets"""
        if target not in self.VALID_TARGETS:
            raise ValueError(f"Invalid target: {target}. Must be one of {self.VALID_TARGETS}")
        pass
    
    def clear_force(self, target: str):
        """Clear force from sensor point"""
        if target not in self.VALID_TARGETS:
            raise ValueError(f"Invalid target: {target}. Must be one of {self.VALID_TARGETS}")
        pass
```

**Force application points (sensor targets only):**
- `left_upper_arm`: Left arm sensor (for arm movement)
- `right_upper_arm`: Right arm sensor (for arm movement)
- `left_thigh`: Left leg sensor (for leg movement)
- `right_thigh`: Right leg sensor (for leg movement)
- **Body rotation/spin**: Achieved through coordinated forces on the 4 sensor points

### Step 7: Refactor StickFigure Class
Restructure `StickFigure` class:

**New structure:**
- Separate physics setup from force application
- Add constraint management subsystem
- Add force application subsystem
- Add limb constraint subsystem
- Keep rendering separate from physics

**Key methods to add/modify:**
- `_apply_tilt_constraints()`: Apply body tilt limits
- `_apply_anti_flip_constraint()`: Prevent body from flipping
- `_check_limb_crossing()`: Detect limb crossings
- `_normalize_limbs()`: Apply corrective forces to limbs
- `_determine_front_back()`: Calculate if front or back is showing based on torso rotation
- `_draw_front_back_indicator()`: Draw "F" or "B" on torso to show orientation
- `apply_force()`: Public method to apply forces to sensor points only (validates target)
- `update()`: Modified to apply all constraints before physics step

### Step 8: Front/Back Indicator Display
Implement visual indicator showing which side of body is facing viewer:

**Front/Back determination:**
- Calculate torso rotation angle relative to world coordinates
- Determine if front (0°) or back (180°) is facing based on angle
- Consider angle ranges: 
  - Front facing: -90° to 90° (torso "up" direction pointing toward viewer)
  - Back facing: 90° to 270° or -270° to -90° (torso "up" direction pointing away)
- Handle angle wrapping (0° = 360°)

**Visual display:**
- Draw letter "F" on torso when front is showing
- Draw letter "B" on torso when back is showing
- Position text at center or slightly offset from torso center
- Use contrasting color (e.g., white text on dark background, or outlined text)
- Rotate text to align with torso orientation for readability
- Use pygame font rendering for clear letter display

**Implementation:**
- Add method `_determine_front_back()`: Returns "F" or "B" based on torso angle
- Add method `_draw_front_back_indicator()`: Renders the letter on torso
- Call in main draw method after drawing torso body

### Step 9: Testing Plan

**Test cases:**

1. **Force sequence loading:**
   - Load JSON file with multiple sequences
   - Verify forces apply at correct times
   - Verify forces fade correctly
   - Verify only valid sensor targets are accepted

2. **Invalid target validation:**
   - Attempt to apply force to "torso" - should raise error
   - Attempt to apply force to invalid target - should raise error
   - Verify only 4 valid targets are accepted

3. **Tilt constraints:**
   - Apply force to sensor points that would tilt body beyond max angle
   - Verify body stops at max tilt
   - Verify restoring force returns body to center

4. **Anti-flip:**
   - Apply strong downward forces to thighs
   - Verify body never flips "ass up"
   - Verify body returns to upright position

5. **Body spin:**
   - Apply opposing horizontal forces to left/right sensor points
   - Verify body rotates 180° (or more) around vertical axis
   - Verify tilt constraints maintained during spin
   - Verify spin achieved through sensor forces, not direct torso torque

6. **Front/Back indicator:**
   - Rotate body 180° and verify "F" changes to "B"
   - Rotate body back and verify "B" changes to "F"
   - Verify letter is clearly visible and properly positioned
   - Verify letter rotates with torso for readability

7. **Limb cross-prevention:**
   - Force left arm to right side using sensor forces
   - Verify corrective force pushes it back
   - Test all limb pairs for crossing

8. **Limb normalization:**
   - Move limbs to correct side but near midline using sensor forces
   - Verify gentle force keeps them in correct zone
   - Test both left and right limbs

### Step 10: Configuration

**Configurable parameters:**
- `max_torso_tilt`: Maximum tilt angle in degrees (default: 45)
- `anti_flip_threshold`: Angle threshold for anti-flip (default: 85 degrees)
- `limb_cross_threshold`: Horizontal distance threshold for crossing detection
- `limb_corrective_force_strength`: Strength of corrective forces
- `spin_force_magnitude`: Magnitude of forces applied to sensor points for spin (default: 100)
- `force_fade_rate`: Rate of force fading (for exponential decay)
- `front_back_indicator_enabled`: Show F/B indicator (default: true)
- `front_back_indicator_color`: Color of F/B text (default: white)
- `front_back_indicator_size`: Font size for F/B text (default: 24)

**Config file location:**
- `config/stick_figure_config.json`
- Or add to existing `config/app_config.json`

## File Structure

```
src/ui/stick_figure.py           # Main StickFigure class (rewritten)
src/ui/stick_figure_forces.py    # ForceSequence, ForceManager classes (new)
src/ui/stick_figure_constraints.py  # Constraint system (new, optional)
config/
  force_sequences.json           # Preloaded force sequences (new)
  stick_figure_config.json       # Physics parameters (new)
```

## Technical Notes

### Physics Library
- Continue using pymunk for physics simulation
- Use pymunk.Body for all body parts
- Use pymunk constraints/joints for connections
- Consider pymunk.RotaryLimitJoint or RotarySpring for tilt constraints
- Or apply manual torques for more control

### Coordinate Systems
- World coordinates: screen space (x, y)
- Body local coordinates: relative to body center and rotation
- Torso orientation: angle relative to vertical (world up = 0°)
- Spin axis: vertical (Z-axis in 3D, but 2D simulation so just angle around center)

### Force Units
- Forces in pymunk units (typically pixels/second² per mass unit)
- Torque in pymunk units (typically pixels²/second² per moment of inertia)
- Need to calibrate force magnitudes based on body masses

### Performance Considerations
- Cross-detection: Only check once per frame, use spatial partitioning if many limbs
- Constraint application: Apply constraints before physics step, not after
- Force sequences: Use efficient data structures, remove expired forces immediately

## Dependencies

- `pymunk`: Physics simulation (already used)
- `json`: For loading force sequences (standard library)
- `math`: For angle calculations (already used)
- `pygame`: For rendering (already used)

## Future Enhancements

- Real-time force feedback from peripheral devices
- Recording/playback of force sequences
- Visual debugging for constraints (show constraint limits, corrective forces)
- Advanced limb IK (inverse kinematics) for more natural movement
- Momentum-based movement (preserve momentum when forces fade)
- Collision response improvements (better bouncing, sliding)

## Summary

This rewrite will:
1. Add JSON-based force sequence system for testing (forces only on 4 sensor points)
2. Implement body tilt and anti-flip constraints
3. Enable controlled 180° body rotation through coordinated sensor forces (not direct torso torque)
4. Prevent limb crossing with automatic correction
5. Add visual front/back indicator (F/B) on torso to show orientation
6. Design input interface for future peripheral integration (validates sensor targets)
7. Maintain clean separation between physics, constraints, and forces

**Important constraints:**
- Forces can ONLY be applied to: `left_upper_arm`, `right_upper_arm`, `left_thigh`, `right_thigh`
- Torso is NOT a valid sensor target
- Body rotation/spin must be achieved through forces on sensor points
- Front/Back indicator helps visualize body orientation during spins and movements

The system will be more robust, controllable, and suitable for interactive use with peripheral input devices.
