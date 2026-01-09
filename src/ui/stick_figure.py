#!/usr/bin/env python3
"""
Stick figure class for animation
Designed as 2D representation of a 3D figure with articulated joints
"""

import pygame
import math
import random
import time


class StickFigure:
    """
    A stick figure with T-shaped body, 2-part arms, and 2-part legs.
    Designed to be animated in 3D space (though rendered in 2D).
    """
    
    def __init__(self, x=0, y=0, scale=1.0):
        """
        Initialize the stick figure
        
        Args:
            x: X position of the figure's center
            y: Y position of the figure's center (top of head)
            scale: Scale factor for the entire figure
        """
        self.x = x
        self.y = y
        self.scale = scale
        
        # Body proportions (in relative units, will be scaled)
        self.head_radius = 15
        self.torso_length = 60
        self.shoulder_width = 50
        self.upper_arm_length = 40
        self.forearm_length = 35
        self.thigh_length = 45
        self.shin_length = 40
        
        # Joint angles (in degrees, for 3D rotation simulation)
        # 0 degrees = default pose
        self.head_angle = 0
        self.torso_angle = 0
        
        # Left arm angles - start with arms hanging down naturally
        self.left_shoulder_angle = 180  # Rotation around shoulder (180 = pointing left)
        self.left_shoulder_lift = 60   # Up/down lift (60 = pointing down at angle)
        self.left_elbow_angle = 0     # Elbow bend
        
        # Right arm angles
        self.right_shoulder_angle = 0  # 0 = pointing right
        self.right_shoulder_lift = 60   # Pointing down at angle
        self.right_elbow_angle = 0
        
        # Left leg angles - start with legs pointing down
        self.left_hip_angle = 0       # Rotation around hip (straight down)
        self.left_hip_lift = 90       # Forward/backward lift (90 = pointing straight down)
        self.left_knee_angle = 0      # Knee bend
        
        # Right leg angles
        self.right_hip_angle = 0
        self.right_hip_lift = 90      # Pointing straight down
        self.right_knee_angle = 0
        
        # Color
        self.color = (255, 255, 255)  # White by default
        self.line_width = 3
        
        # Acceleration visualization
        self.show_accelerations = False
        self.accel_circle_radius = 8
        self.accel_arrow_length_scale = 2.0  # Scale factor for arrow length
        self.accel_arrow_color = (255, 200, 0)  # Orange/yellow for acceleration arrows
        self.accel_circle_color = (0, 255, 255)  # Cyan for acceleration point circles
        
        # Physics simulation for acceleration-based movement
        # Accelerations (can be set from sensors later)
        # Format: (x_accel, y_accel, z_accel) in degrees per second squared
        self.left_upper_arm_accel = (0.0, 0.0, 0.0)
        self.right_upper_arm_accel = (0.0, 0.0, 0.0)
        self.left_upper_leg_accel = (0.0, 0.0, 0.0)
        self.right_upper_leg_accel = (0.0, 0.0, 0.0)
        
        # Target accelerations for smooth interpolation
        self.left_upper_arm_target_accel = (0.0, 0.0, 0.0)
        self.right_upper_arm_target_accel = (0.0, 0.0, 0.0)
        self.left_upper_leg_target_accel = (0.0, 0.0, 0.0)
        self.right_upper_leg_target_accel = (0.0, 0.0, 0.0)
        
        # Velocities (angular velocities in degrees per second)
        self.left_shoulder_vel = 0.0
        self.left_shoulder_lift_vel = 0.0
        self.right_shoulder_vel = 0.0
        self.right_shoulder_lift_vel = 0.0
        self.left_hip_vel = 0.0
        self.left_hip_lift_vel = 0.0
        self.right_hip_vel = 0.0
        self.right_hip_lift_vel = 0.0
        
        # Elbow and knee velocities (for lower limb parts - forearms and shins)
        # These will lag behind upper limb movement due to inertia and drag
        self.left_elbow_vel = 0.0
        self.right_elbow_vel = 0.0
        self.left_knee_vel = 0.0
        self.right_knee_vel = 0.0
        
        # Physics parameters
        self.damping = 0.98  # Velocity damping factor (0-1, higher = less damping) - increased for more movement
        self.leg_damping = 0.96  # Additional damping for lower limbs (legs) - makes them slower/more sluggish
        self.accel_scale = 1.0  # Scale factor for acceleration to velocity conversion - increased for more responsiveness
        self.max_velocity = 360.0  # Maximum angular velocity in degrees per second - increased for more movement
        self.accel_interpolation_speed = 5.0  # Speed of acceleration interpolation (higher = faster, smoother transitions)
        
        # Lower limb physics parameters (forearms and shins)
        self.lower_limb_inertia = 0.4  # How much lower limb responds to upper limb movement (0-1, lower = more lag)
        self.lower_limb_damping = 0.88  # Drag on lower limbs (creates lag/bend effect, lower = more drag)
        self.lower_limb_max_bend = 160.0  # Maximum bend angle in degrees (for elbows and knees)
        self.lower_limb_response_speed = 30.0  # Speed at which lower limbs respond to upper limb movement
        
        # Random acceleration parameters
        self.use_random_accel = True
        self.random_accel_strength = 30.0  # Maximum random acceleration (increased for more movement)
        self.random_accel_change_rate = 0.15  # Probability of changing acceleration each frame (increased)
        self.last_random_update = 0
        self.random_update_interval = 50  # Update random accelerations every N milliseconds (more frequent)
        
    def set_position(self, x, y):
        """Set the position of the stick figure"""
        self.x = x
        self.y = y
    
    def set_scale(self, scale):
        """Set the scale of the stick figure"""
        self.scale = scale
    
    def set_color(self, color):
        """Set the color of the stick figure"""
        self.color = color
    
    def set_show_accelerations(self, show):
        """Enable or disable acceleration visualization"""
        self.show_accelerations = show
    
    def _apply_scale(self, value):
        """Apply scale to a value"""
        return value * self.scale
    
    def _deg_to_rad(self, degrees):
        """Convert degrees to radians"""
        return math.radians(degrees)
    
    def _calculate_joint_position(self, start_x, start_y, length, angle_deg, lift_deg=0):
        """
        Calculate end position of a limb segment in 2D
        
        Args:
            start_x, start_y: Starting position
            length: Length of segment
            angle_deg: Horizontal rotation (0 = right, 180 = left)
            lift_deg: Vertical tilt (0 = horizontal, 90 = down, -90 = up)
        
        Returns:
            (end_x, end_y) tuple
        """
        # Scale the length
        scaled_length = self._apply_scale(length)
        
        # Convert angles to radians
        angle_rad = self._deg_to_rad(angle_deg)
        lift_rad = self._deg_to_rad(lift_deg)
        
        # Simple 2D projection:
        # - angle_deg controls horizontal direction (left/right)
        # - lift_deg controls vertical direction (up/down)
        # 
        # Project the 3D direction onto 2D:
        # X: horizontal component from angle, scaled by how horizontal the limb is
        # Y: vertical component from lift
        
        # Horizontal component: cos(angle) gives left/right, scaled by cos(lift) for horizontal tilt
        x_offset = scaled_length * math.cos(angle_rad) * math.cos(lift_rad)
        
        # Vertical component: sin(lift) gives up/down
        y_offset = scaled_length * math.sin(lift_rad)
        
        return (start_x + x_offset, start_y + y_offset)
    
    def get_joint_positions(self):
        """
        Calculate all joint positions based on current angles
        Returns a dictionary with all joint coordinates
        """
        positions = {}
        
        # Head center
        head_y = self.y + self._apply_scale(self.head_radius)
        positions['head_center'] = (self.x, head_y)
        
        # Neck (top of torso)
        neck_y = head_y + self._apply_scale(self.head_radius)
        positions['neck'] = (self.x, neck_y)
        
        # Shoulders (center of shoulder line)
        shoulder_y = neck_y + self._apply_scale(self.torso_length * 0.3)
        positions['shoulder_center'] = (self.x, shoulder_y)
        
        # Left and right shoulders
        shoulder_half_width = self._apply_scale(self.shoulder_width / 2)
        positions['left_shoulder'] = (self.x - shoulder_half_width, shoulder_y)
        positions['right_shoulder'] = (self.x + shoulder_half_width, shoulder_y)
        
        # Waist (bottom of torso)
        waist_y = shoulder_y + self._apply_scale(self.torso_length * 0.7)
        positions['waist'] = (self.x, waist_y)
        
        # Left hip (slightly left of center)
        hip_offset = self._apply_scale(8)
        positions['left_hip'] = (self.x - hip_offset, waist_y)
        positions['right_hip'] = (self.x + hip_offset, waist_y)
        
        # Left arm joints
        left_elbow = self._calculate_joint_position(
            positions['left_shoulder'][0],
            positions['left_shoulder'][1],
            self.upper_arm_length,
            self.left_shoulder_angle,
            self.left_shoulder_lift
        )
        positions['left_elbow'] = left_elbow
        
        # Calculate forearm direction relative to upper arm
        # Elbow angle bends the forearm relative to the upper arm direction
        forearm_angle = self.left_shoulder_angle
        forearm_lift = self.left_shoulder_lift + self.left_elbow_angle  # Elbow bend adds to lift angle
        left_wrist = self._calculate_joint_position(
            left_elbow[0],
            left_elbow[1],
            self.forearm_length,
            forearm_angle,
            forearm_lift
        )
        positions['left_wrist'] = left_wrist
        
        # Right arm joints
        right_elbow = self._calculate_joint_position(
            positions['right_shoulder'][0],
            positions['right_shoulder'][1],
            self.upper_arm_length,
            self.right_shoulder_angle,
            self.right_shoulder_lift
        )
        positions['right_elbow'] = right_elbow
        
        forearm_angle = self.right_shoulder_angle
        forearm_lift = self.right_shoulder_lift + self.right_elbow_angle
        right_wrist = self._calculate_joint_position(
            right_elbow[0],
            right_elbow[1],
            self.forearm_length,
            forearm_angle,
            forearm_lift
        )
        positions['right_wrist'] = right_wrist
        
        # Left leg joints
        left_knee = self._calculate_joint_position(
            positions['left_hip'][0],
            positions['left_hip'][1],
            self.thigh_length,
            self.left_hip_angle,
            self.left_hip_lift
        )
        positions['left_knee'] = left_knee
        
        shin_angle = self.left_hip_angle
        shin_lift = self.left_hip_lift + self.left_knee_angle  # Knee bend adds to lift angle
        left_ankle = self._calculate_joint_position(
            left_knee[0],
            left_knee[1],
            self.shin_length,
            shin_angle,
            shin_lift
        )
        positions['left_ankle'] = left_ankle
        
        # Right leg joints
        right_knee = self._calculate_joint_position(
            positions['right_hip'][0],
            positions['right_hip'][1],
            self.thigh_length,
            self.right_hip_angle,
            self.right_hip_lift
        )
        positions['right_knee'] = right_knee
        
        shin_angle = self.right_hip_angle
        shin_lift = self.right_hip_lift + self.right_knee_angle
        right_ankle = self._calculate_joint_position(
            right_knee[0],
            right_knee[1],
            self.shin_length,
            shin_angle,
            shin_lift
        )
        positions['right_ankle'] = right_ankle
        
        return positions
    
    def draw(self, surface):
        """
        Draw the stick figure on the given surface
        
        Args:
            surface: Pygame surface to draw on
        """
        positions = self.get_joint_positions()
        
        # Draw head (circle)
        head_center = positions['head_center']
        head_radius = self._apply_scale(self.head_radius)
        pygame.draw.circle(surface, self.color, 
                          (int(head_center[0]), int(head_center[1])), 
                          int(head_radius), self.line_width)
        
        # Draw torso (vertical line from neck to waist)
        pygame.draw.line(surface, self.color,
                        positions['neck'],
                        positions['waist'],
                        self.line_width)
        
        # Draw shoulders (horizontal line)
        pygame.draw.line(surface, self.color,
                        positions['left_shoulder'],
                        positions['right_shoulder'],
                        self.line_width)
        
        # Draw left arm (upper arm + forearm)
        pygame.draw.line(surface, self.color,
                        positions['left_shoulder'],
                        positions['left_elbow'],
                        self.line_width)
        pygame.draw.line(surface, self.color,
                        positions['left_elbow'],
                        positions['left_wrist'],
                        self.line_width)
        
        # Draw right arm (upper arm + forearm)
        pygame.draw.line(surface, self.color,
                        positions['right_shoulder'],
                        positions['right_elbow'],
                        self.line_width)
        pygame.draw.line(surface, self.color,
                        positions['right_elbow'],
                        positions['right_wrist'],
                        self.line_width)
        
        # Draw left leg (thigh + shin)
        pygame.draw.line(surface, self.color,
                        positions['left_hip'],
                        positions['left_knee'],
                        self.line_width)
        pygame.draw.line(surface, self.color,
                        positions['left_knee'],
                        positions['left_ankle'],
                        self.line_width)
        
        # Draw right leg (thigh + shin)
        pygame.draw.line(surface, self.color,
                        positions['right_hip'],
                        positions['right_knee'],
                        self.line_width)
        pygame.draw.line(surface, self.color,
                        positions['right_knee'],
                        positions['right_ankle'],
                        self.line_width)
        
        # Draw acceleration visualizations if enabled
        if self.show_accelerations:
            self._draw_accelerations(surface, positions)
        
        # Optional: Draw joint points for debugging
        # Uncomment to visualize joints
        # for joint_name, pos in positions.items():
        #     pygame.draw.circle(surface, (255, 0, 0), 
        #                       (int(pos[0]), int(pos[1])), 3)
    
    def _draw_accelerations(self, surface, positions):
        """
        Draw acceleration visualization: circles at acceleration points and arrows for 3D accelerations
        
        Args:
            surface: Pygame surface to draw on
            positions: Dictionary of joint positions
        """
        # Calculate midpoints of upper arms and upper legs (where accelerations are applied)
        left_upper_arm_mid = (
            (positions['left_shoulder'][0] + positions['left_elbow'][0]) / 2,
            (positions['left_shoulder'][1] + positions['left_elbow'][1]) / 2
        )
        right_upper_arm_mid = (
            (positions['right_shoulder'][0] + positions['right_elbow'][0]) / 2,
            (positions['right_shoulder'][1] + positions['right_elbow'][1]) / 2
        )
        left_upper_leg_mid = (
            (positions['left_hip'][0] + positions['left_knee'][0]) / 2,
            (positions['left_hip'][1] + positions['left_knee'][1]) / 2
        )
        right_upper_leg_mid = (
            (positions['right_hip'][0] + positions['right_knee'][0]) / 2,
            (positions['right_hip'][1] + positions['right_knee'][1]) / 2
        )
        
        # Draw circles and arrows for each acceleration point
        self._draw_accel_point(surface, left_upper_arm_mid, self.left_upper_arm_accel)
        self._draw_accel_point(surface, right_upper_arm_mid, self.right_upper_arm_accel)
        self._draw_accel_point(surface, left_upper_leg_mid, self.left_upper_leg_accel)
        self._draw_accel_point(surface, right_upper_leg_mid, self.right_upper_leg_accel)
    
    def _draw_accel_point(self, surface, position, accel):
        """
        Draw a circle at the acceleration point and arrows for 3D acceleration
        
        Args:
            surface: Pygame surface to draw on
            position: (x, y) position of the acceleration point
            accel: (x_accel, y_accel, z_accel) tuple
        """
        x, y = int(position[0]), int(position[1])
        accel_x, accel_y, accel_z = accel
        
        # Draw circle at acceleration point
        circle_radius = int(self._apply_scale(self.accel_circle_radius))
        pygame.draw.circle(surface, self.accel_circle_color, (x, y), circle_radius, 2)
        
        # Calculate arrow length based on acceleration magnitude
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude < 0.1:  # Don't draw if acceleration is too small
            return
        
        # Scale arrow length
        base_length = self._apply_scale(20)  # Base arrow length
        arrow_length = base_length * accel_magnitude / 50.0 * self.accel_arrow_length_scale
        arrow_length = max(5, min(arrow_length, base_length * 3))  # Clamp arrow length
        
        # Draw X component arrow (horizontal, red)
        if abs(accel_x) > 0.1:
            x_arrow_length = arrow_length * abs(accel_x) / accel_magnitude
            x_direction = 1 if accel_x > 0 else -1
            end_x = x + x_arrow_length * x_direction
            self._draw_arrow(surface, (x, y), (end_x, y), (255, 100, 100), 2)
        
        # Draw Y component arrow (vertical, green)
        if abs(accel_y) > 0.1:
            y_arrow_length = arrow_length * abs(accel_y) / accel_magnitude
            y_direction = 1 if accel_y > 0 else -1
            end_y = y + y_arrow_length * y_direction
            self._draw_arrow(surface, (x, y), (x, end_y), (100, 255, 100), 2)
        
        # Draw Z component arrow (diagonal/perpendicular, blue)
        # Z represents depth, so we'll show it as a diagonal arrow
        if abs(accel_z) > 0.1:
            z_arrow_length = arrow_length * abs(accel_z) / accel_magnitude
            z_direction = 1 if accel_z > 0 else -1
            # Show Z as a diagonal arrow (45 degrees)
            end_x = x + z_arrow_length * z_direction * math.cos(math.pi / 4)
            end_y = y + z_arrow_length * z_direction * math.sin(math.pi / 4)
            self._draw_arrow(surface, (x, y), (end_x, end_y), (100, 100, 255), 2)
    
    def _draw_arrow(self, surface, start, end, color, width):
        """
        Draw an arrow from start to end
        
        Args:
            surface: Pygame surface to draw on
            start: (x, y) start position
            end: (x, y) end position
            color: RGB color tuple
            width: Line width
        """
        # Draw the arrow line
        pygame.draw.line(surface, color, start, end, width)
        
        # Calculate arrowhead
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle = math.atan2(dy, dx)
        
        # Arrowhead size
        arrowhead_length = 8
        arrowhead_angle = math.pi / 6  # 30 degrees
        
        # Calculate arrowhead points
        arrowhead_x1 = end[0] - arrowhead_length * math.cos(angle - arrowhead_angle)
        arrowhead_y1 = end[1] - arrowhead_length * math.sin(angle - arrowhead_angle)
        arrowhead_x2 = end[0] - arrowhead_length * math.cos(angle + arrowhead_angle)
        arrowhead_y2 = end[1] - arrowhead_length * math.sin(angle + arrowhead_angle)
        
        # Draw arrowhead
        pygame.draw.polygon(surface, color, [
            end,
            (arrowhead_x1, arrowhead_y1),
            (arrowhead_x2, arrowhead_y2)
        ])
    
    def set_pose(self, **kwargs):
        """
        Set multiple joint angles at once
        
        Example:
            figure.set_pose(
                left_shoulder_angle=45,
                right_shoulder_angle=-45,
                left_elbow_angle=90,
                right_elbow_angle=90
            )
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def reset_pose(self):
        """Reset all angles to default (standing pose)"""
        self.head_angle = 0
        self.torso_angle = 0
        
        self.left_shoulder_angle = 0
        self.left_shoulder_lift = 0
        self.left_elbow_angle = 0
        
        self.right_shoulder_angle = 0
        self.right_shoulder_lift = 0
        self.right_elbow_angle = 0
        
        self.left_hip_angle = 0
        self.left_hip_lift = 0
        self.left_knee_angle = 0
        
        self.right_hip_angle = 0
        self.right_hip_lift = 0
        self.right_knee_angle = 0
        
        # Reset velocities
        self.left_shoulder_vel = 0.0
        self.left_shoulder_lift_vel = 0.0
        self.right_shoulder_vel = 0.0
        self.right_shoulder_lift_vel = 0.0
        self.left_hip_vel = 0.0
        self.left_hip_lift_vel = 0.0
        self.right_hip_vel = 0.0
        self.right_hip_lift_vel = 0.0
        self.left_elbow_vel = 0.0
        self.right_elbow_vel = 0.0
        self.left_knee_vel = 0.0
        self.right_knee_vel = 0.0
    
    def set_acceleration(self, limb, accel):
        """
        Set target acceleration for a limb (for future sensor integration)
        The acceleration will smoothly interpolate to this target
        
        Args:
            limb: 'left_upper_arm', 'right_upper_arm', 'left_upper_leg', 'right_upper_leg'
            accel: Tuple of (x, y, z) accelerations in degrees per second squared
        """
        if limb == 'left_upper_arm':
            self.left_upper_arm_target_accel = accel
        elif limb == 'right_upper_arm':
            self.right_upper_arm_target_accel = accel
        elif limb == 'left_upper_leg':
            self.left_upper_leg_target_accel = accel
        elif limb == 'right_upper_leg':
            self.right_upper_leg_target_accel = accel
    
    def _generate_random_accelerations(self):
        """Generate random target accelerations for smooth interpolation (will be replaced by sensor data)"""
        current_time = time.time() * 1000  # Convert to milliseconds
        
        # Update random target accelerations periodically
        if current_time - self.last_random_update > self.random_update_interval:
            # Randomly change target accelerations (these will be smoothly interpolated to)
            if random.random() < self.random_accel_change_rate:
                # Left upper arm: affects shoulder angle and lift
                self.left_upper_arm_target_accel = (
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength * 0.5, self.random_accel_strength * 0.5)
                )
            
            if random.random() < self.random_accel_change_rate:
                # Right upper arm
                self.right_upper_arm_target_accel = (
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength * 0.5, self.random_accel_strength * 0.5)
                )
            
            if random.random() < self.random_accel_change_rate:
                # Left upper leg: affects hip angle and lift
                self.left_upper_leg_target_accel = (
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength * 0.5, self.random_accel_strength * 0.5)
                )
            
            if random.random() < self.random_accel_change_rate:
                # Right upper leg
                self.right_upper_leg_target_accel = (
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength, self.random_accel_strength),
                    random.uniform(-self.random_accel_strength * 0.5, self.random_accel_strength * 0.5)
                )
            
            self.last_random_update = current_time
    
    def _lerp(self, a, b, t):
        """Linear interpolation between two values"""
        return a + (b - a) * t
    
    def _lerp_tuple(self, a, b, t):
        """Linear interpolation between two tuples"""
        return tuple(self._lerp(a[i], b[i], t) for i in range(len(a)))
    
    def _smooth_accelerations(self, dt):
        """Smoothly interpolate current accelerations toward target accelerations"""
        # Calculate interpolation factor based on time and speed
        interp_factor = 1.0 - math.exp(-self.accel_interpolation_speed * dt)
        
        # Interpolate each acceleration toward its target
        self.left_upper_arm_accel = self._lerp_tuple(
            self.left_upper_arm_accel,
            self.left_upper_arm_target_accel,
            interp_factor
        )
        self.right_upper_arm_accel = self._lerp_tuple(
            self.right_upper_arm_accel,
            self.right_upper_arm_target_accel,
            interp_factor
        )
        self.left_upper_leg_accel = self._lerp_tuple(
            self.left_upper_leg_accel,
            self.left_upper_leg_target_accel,
            interp_factor
        )
        self.right_upper_leg_accel = self._lerp_tuple(
            self.right_upper_leg_accel,
            self.right_upper_leg_target_accel,
            interp_factor
        )
    
    def update(self, dt):
        """
        Update the stick figure based on accelerations
        
        Args:
            dt: Delta time in seconds (time since last update)
        """
        # Generate random target accelerations if enabled
        if self.use_random_accel:
            self._generate_random_accelerations()
        
        # Smoothly interpolate accelerations toward targets
        self._smooth_accelerations(dt)
        
        # Apply accelerations to left upper arm (affects shoulder)
        # X acceleration affects horizontal rotation (shoulder_angle)
        # Y acceleration affects vertical lift (shoulder_lift)
        self.left_shoulder_vel += self.left_upper_arm_accel[0] * self.accel_scale * dt
        self.left_shoulder_lift_vel += self.left_upper_arm_accel[1] * self.accel_scale * dt
        
        # Apply accelerations to right upper arm
        self.right_shoulder_vel += self.right_upper_arm_accel[0] * self.accel_scale * dt
        self.right_shoulder_lift_vel += self.right_upper_arm_accel[1] * self.accel_scale * dt
        
        # Apply accelerations to left upper leg (affects hip)
        self.left_hip_vel += self.left_upper_leg_accel[0] * self.accel_scale * dt
        self.left_hip_lift_vel += self.left_upper_leg_accel[1] * self.accel_scale * dt
        
        # Apply accelerations to right upper leg
        self.right_hip_vel += self.right_upper_leg_accel[0] * self.accel_scale * dt
        self.right_hip_lift_vel += self.right_upper_leg_accel[1] * self.accel_scale * dt
        
        # Apply damping to velocities
        # Arms use normal damping
        self.left_shoulder_vel *= self.damping
        self.left_shoulder_lift_vel *= self.damping
        self.right_shoulder_vel *= self.damping
        self.right_shoulder_lift_vel *= self.damping
        
        # Legs use additional drag (more damping) for slower, more sluggish movement
        self.left_hip_vel *= self.damping * self.leg_damping
        self.left_hip_lift_vel *= self.damping * self.leg_damping
        self.right_hip_vel *= self.damping * self.leg_damping
        self.right_hip_lift_vel *= self.damping * self.leg_damping
        
        # Lower limb physics: forearms and shins lag behind upper arms and thighs due to inertia and drag
        # This creates natural bending at elbows and knees when limbs are moving
        
        # Elbow physics: forearms lag behind upper arm movement
        # When the upper arm moves up/down, the forearm should lag behind, creating a bend
        # The target velocity is based on how fast the upper arm is moving
        # Negative because when arm goes up (positive lift_vel), elbow should bend (positive angle)
        left_elbow_target_vel = -self.left_shoulder_lift_vel * self.lower_limb_inertia
        right_elbow_target_vel = -self.right_shoulder_lift_vel * self.lower_limb_inertia
        
        # Also respond to horizontal movement (shoulder angle velocity)
        left_elbow_target_vel += abs(self.left_shoulder_vel) * self.lower_limb_inertia * 0.3
        right_elbow_target_vel += abs(self.right_shoulder_vel) * self.lower_limb_inertia * 0.3
        
        # Interpolate elbow velocity toward target (creates lag/inertia effect)
        response_factor = (1.0 - self.lower_limb_damping) * self.lower_limb_response_speed * dt
        self.left_elbow_vel += (left_elbow_target_vel - self.left_elbow_vel) * response_factor
        self.right_elbow_vel += (right_elbow_target_vel - self.right_elbow_vel) * response_factor
        
        # Apply drag to elbow velocities (forearms resist movement)
        self.left_elbow_vel *= self.lower_limb_damping
        self.right_elbow_vel *= self.lower_limb_damping
        
        # Knee physics: shins lag behind thigh movement
        # When the thigh moves up/down, the shin should lag behind, creating a bend
        left_knee_target_vel = -self.left_hip_lift_vel * self.lower_limb_inertia
        right_knee_target_vel = -self.right_hip_lift_vel * self.lower_limb_inertia
        
        # Also respond to horizontal movement (hip angle velocity)
        left_knee_target_vel += abs(self.left_hip_vel) * self.lower_limb_inertia * 0.3
        right_knee_target_vel += abs(self.right_hip_vel) * self.lower_limb_inertia * 0.3
        
        # Interpolate knee velocity toward target (creates lag/inertia effect)
        self.left_knee_vel += (left_knee_target_vel - self.left_knee_vel) * response_factor
        self.right_knee_vel += (right_knee_target_vel - self.right_knee_vel) * response_factor
        
        # Apply drag to knee velocities (shins resist movement)
        self.left_knee_vel *= self.lower_limb_damping
        self.right_knee_vel *= self.lower_limb_damping
        
        # Clamp velocities to maximum
        self.left_shoulder_vel = max(-self.max_velocity, min(self.max_velocity, self.left_shoulder_vel))
        self.left_shoulder_lift_vel = max(-self.max_velocity, min(self.max_velocity, self.left_shoulder_lift_vel))
        self.right_shoulder_vel = max(-self.max_velocity, min(self.max_velocity, self.right_shoulder_vel))
        self.right_shoulder_lift_vel = max(-self.max_velocity, min(self.max_velocity, self.right_shoulder_lift_vel))
        self.left_hip_vel = max(-self.max_velocity, min(self.max_velocity, self.left_hip_vel))
        self.left_hip_lift_vel = max(-self.max_velocity, min(self.max_velocity, self.left_hip_lift_vel))
        self.right_hip_vel = max(-self.max_velocity, min(self.max_velocity, self.right_hip_vel))
        self.right_hip_lift_vel = max(-self.max_velocity, min(self.max_velocity, self.right_hip_lift_vel))
        
        # Update angles based on velocities
        self.left_shoulder_angle += self.left_shoulder_vel * dt
        self.left_shoulder_lift += self.left_shoulder_lift_vel * dt
        self.right_shoulder_angle += self.right_shoulder_vel * dt
        self.right_shoulder_lift += self.right_shoulder_lift_vel * dt
        self.left_hip_angle += self.left_hip_vel * dt
        self.left_hip_lift += self.left_hip_lift_vel * dt
        self.right_hip_angle += self.right_hip_vel * dt
        self.right_hip_lift += self.right_hip_lift_vel * dt
        
        # Update elbow and knee angles based on their velocities
        # These angles represent the bend between upper and lower parts
        self.left_elbow_angle += self.left_elbow_vel * dt
        self.right_elbow_angle += self.right_elbow_vel * dt
        self.left_knee_angle += self.left_knee_vel * dt
        self.right_knee_angle += self.right_knee_vel * dt
        
        # Clamp elbow and knee angles to reasonable range
        # Positive angles = bending (forearm/shin lagging behind)
        self.left_elbow_angle = max(0, min(self.lower_limb_max_bend, self.left_elbow_angle))
        self.right_elbow_angle = max(0, min(self.lower_limb_max_bend, self.right_elbow_angle))
        self.left_knee_angle = max(0, min(self.lower_limb_max_bend, self.left_knee_angle))
        self.right_knee_angle = max(0, min(self.lower_limb_max_bend, self.right_knee_angle))
        
        # Keep angles in reasonable ranges (optional, can be adjusted)
        # Allow arms to move more freely
        self.left_shoulder_lift = max(-45, min(135, self.left_shoulder_lift))
        self.right_shoulder_lift = max(-45, min(135, self.right_shoulder_lift))
        # Legs should stay pointing down (between 45 and 135 degrees)
        self.left_hip_lift = max(45, min(135, self.left_hip_lift))
        self.right_hip_lift = max(45, min(135, self.right_hip_lift))
