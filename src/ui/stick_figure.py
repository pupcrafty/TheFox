#!/usr/bin/env python3
"""
Stick figure class with pymunk physics simulation
All parts are free-floating but connected with joints
Vector forces applied to upper arms and thighs (where sensors will be)
"""

import pygame
import math
import random
import time
import pymunk


class StickFigure:
    """
    A stick figure with T-shaped body, 2-part arms, and 2-part legs.
    Uses pymunk physics where all parts are free-floating but connected with joints.
    Forces are applied to upper arms and thighs based on sensor data.
    """
    
    def __init__(self, x=0, y=0, scale=1.0, space=None):
        """
        Initialize the stick figure with pymunk physics
        
        Args:
            x: X position of the figure's center (neck/shoulders)
            y: Y position of the figure's center
            scale: Scale factor for the entire figure
            space: pymunk.Space object (creates one if None)
        """
        self.x = x
        self.y = y
        self.scale = scale
        
        # Create or use provided physics space
        if space is None:
            self.space = pymunk.Space()
            self.space.gravity = (0, 150)  # Gravity pointing down (initial value, will be updated in update method)
        else:
            self.space = space
        self.owns_space = space is None
        
        # Body proportions (in relative units, will be scaled)
        self.head_radius = 15
        self.torso_length = 60
        self.shoulder_width = 50
        self.upper_arm_length = 40
        self.forearm_length = 35
        self.thigh_length = 45
        self.shin_length = 40
        
        # Mass for each body segment
        self.mass = 1.0
        self.moment_multiplier = 100.0  # Moment of inertia multiplier
        
        # Limb thickness for collision physics
        self.limb_thickness = 12.0  # Thickness/radius of limb segments (increased for better collision)
        self.collision_buffer_zone = 6.0  # Additional buffer zone around limbs for collision (prevents overlap) - increased
        self.max_joint_distance_factor = 1.2  # Maximum distance joints can stretch (120% of rest length)
        
        # Collision groups: limbs can collide with each other but not with directly connected parts
        # Each part gets a unique category bit
        self.collision_categories = {
            'head': 0x1,
            'torso': 0x2,
            'shoulders': 0x4,
            'left_upper_arm': 0x8,
            'right_upper_arm': 0x10,
            'left_forearm': 0x20,
            'right_forearm': 0x40,
            'left_thigh': 0x80,
            'right_thigh': 0x100,
            'left_shin': 0x200,
            'right_shin': 0x400,
        }
        
        # Physics bodies and joints storage
        self.bodies = {}
        self.joints = {}
        self.constraints = {}
        self.limb_lengths = {}  # Store limb lengths for easier access
        
        # Color
        self.color = (255, 255, 255)  # White by default
        self.line_width = 3
        
        # Acceleration visualization
        self.show_accelerations = False
        self.accel_circle_radius = 8
        self.accel_arrow_length_scale = 2.0
        self.accel_arrow_color = (255, 200, 0)
        self.accel_circle_color = (0, 255, 255)
        
        # Torso rotation visualization
        self.show_torso_rotation = False
        self.rotation_arc_radius = 50
        self.rotation_arc_color = (255, 150, 150)  # Light red
        self.rotation_limit_color = (150, 150, 255)  # Light blue for limits
        
        # Forces applied to upper arms and thighs (from sensors)
        # Format: (fx, fy, torque) in physics units
        self.left_upper_arm_force = (0.0, 0.0, 0.0)
        self.right_upper_arm_force = (0.0, 0.0, 0.0)
        self.left_upper_leg_force = (0.0, 0.0, 0.0)
        self.right_upper_leg_force = (0.0, 0.0, 0.0)
        
        # Target forces for smooth interpolation
        self.left_upper_arm_target_force = (0.0, 0.0, 0.0)
        self.right_upper_arm_target_force = (0.0, 0.0, 0.0)
        self.left_upper_leg_target_force = (0.0, 0.0, 0.0)
        self.right_upper_leg_target_force = (0.0, 0.0, 0.0)
        
        # Physics parameters
        self.damping = 0.96  # Velocity damping (reduced damping for more responsiveness to forces)
        self.force_interpolation_speed = 5.0  # Speed of force interpolation
        self.force_scale = 1200.0  # Scale factor for converting sensor data to forces (increased for better response)
        
        # Gravity and normalizing force parameters
        self.gravity_strength = 150.0  # Gravity in pixels per second squared (downward) - reduced for lighter gravity
        self.normalizing_force_strength = 150.0  # Strength of normalizing force (increased for more control)
        self.center_restore_force = 300.0  # Force to restore center position (stronger to keep centered)
        self.apply_normalizing_forces = True  # Enable/disable normalizing forces
        self.max_velocity = 400.0  # Maximum velocity before normalization (further reduced for stability)
        self.max_angular_velocity = 8.0  # Maximum angular velocity (radians per second) - further reduced
        
        # Random force parameters (for testing, will be replaced by sensor data)
        self.use_random_forces = True
        self.random_force_strength = 150.0  # Further reduced to prevent excessive forces and instability
        self.random_force_change_rate = 0.10  # Reduced for smoother changes
        self.last_random_update = 0
        self.random_update_interval = 50
        
        # Rest pose parameters - prevents drooping by maintaining a natural upright pose
        self.maintain_rest_pose = True  # Enable/disable rest pose maintenance
        self.rest_pose_strength = 200.0  # Strength of restorative forces toward rest pose
        self.rest_angles = {}  # Will store target rest angles for each body
        
        # Build the physics structure
        self._build_physics_structure()
        
        # Set initial pose after all bodies are created
        # The _set_initial_pose will position and orient all bodies
        self._set_initial_pose()
    
    def _build_physics_structure(self):
        """Build the physics bodies and joints for the stick figure"""
        # Calculate scaled dimensions
        scaled_head_radius = self._apply_scale(self.head_radius)
        scaled_torso = self._apply_scale(self.torso_length)
        scaled_shoulder_width = self._apply_scale(self.shoulder_width / 2)
        scaled_upper_arm = self._apply_scale(self.upper_arm_length)
        scaled_forearm = self._apply_scale(self.forearm_length)
        scaled_thigh = self._apply_scale(self.thigh_length)
        scaled_shin = self._apply_scale(self.shin_length)
        
        # Neck/Torso connection point (where shoulders are)
        # Calculate where torso center should be
        torso_center_y = self.y + scaled_head_radius + scaled_torso / 2
        
        # Create a static pivot point at torso center to keep body centered but allow rotation
        static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        static_body.position = (self.x, torso_center_y)  # Pivot at torso center
        self.bodies['center_pivot'] = static_body
        
        # Torso (inverted isosceles triangle - T-shaped body)
        # Triangle: wide at top (shoulders/base) tapering down to point at bottom (hips/apex)
        torso_mass = self.mass * 1.5
        # Triangle vertices relative to body center (local coordinates)
        # Base of triangle is at top (shoulders), apex is at bottom (hips)
        triangle_width_top = scaled_shoulder_width  # Wide at top (shoulders - base of triangle)
        triangle_apex_bottom = scaled_torso/2  # Bottom point (hips - apex of triangle)
        triangle_base_top = -scaled_torso/2  # Top base line (shoulders)
        
        # Create triangle vertices in counterclockwise order for pymunk:
        # top left (left shoulder) -> top right (right shoulder) -> bottom point (hips) -> back to top left
        triangle_vertices = [
            (-triangle_width_top/2, triangle_base_top),  # Left shoulder (top left corner)
            (triangle_width_top/2, triangle_base_top),  # Right shoulder (top right corner)
            (0, triangle_apex_bottom),  # Hips (bottom point/apex)
        ]
        # Triangle: wide at top (shoulders/base), narrows to point at bottom (hips/apex)
        
        # Calculate moment of inertia for triangle
        torso_moment = pymunk.moment_for_poly(torso_mass, triangle_vertices, (0, 0))
        torso_body = pymunk.Body(torso_mass, torso_moment)
        torso_shape = pymunk.Poly(torso_body, triangle_vertices)
        torso_shape.friction = 0.7
        torso_shape.density = 0.5
        # Torso can collide with all limbs but not head/thighs (directly connected)
        # No separate shoulders body now - triangle base IS the shoulders
        torso_category = self.collision_categories['torso']
        torso_mask = 0xFFFF & ~(self.collision_categories['head'] | 
                                self.collision_categories['left_thigh'] | self.collision_categories['right_thigh'] |
                                self.collision_categories['left_upper_arm'] | self.collision_categories['right_upper_arm'])
        torso_shape.filter = pymunk.ShapeFilter(categories=torso_category, mask=torso_mask)
        self.space.add(torso_body, torso_shape)
        self.bodies['torso'] = torso_body
        torso_body.position = (self.x, torso_center_y)  # Torso center at pivot point
        torso_body.angle = math.pi / 2  # Vertical (90 degrees)
        
        # Store initial angle and rotation limits for torso
        self.torso_initial_angle = math.pi / 2  # 90 degrees (vertical)
        self.torso_max_rotation = math.radians(100)  # ±100 degrees from initial
        
        # Shoulder rotation limits (to prevent arms wrapping around body)
        # These are relative to the arm's initial angle
        self.left_shoulder_max_rotation = math.radians(90)  # ±90 degrees from initial (sensible range)
        self.right_shoulder_max_rotation = math.radians(90)  # ±90 degrees from initial (sensible range)
        
        # Connect torso to center pivot at torso center (allows rotation but keeps centered)
        # Both anchor points are at (0, 0) in their local coordinates (center of each body)
        pivot_joint = pymunk.PinJoint(static_body, torso_body, (0, 0), (0, 0))
        self.space.add(pivot_joint)
        self.joints['torso_pivot'] = pivot_joint
        
        # Head (circle body at top) - positioned above torso
        # Lighter head mass to reduce gravitational pull
        head_mass = self.mass * 0.3  # Reduced from 0.5 to 0.3 for stability
        # Add collision buffer zone to head radius
        head_collision_radius = scaled_head_radius + self._apply_scale(self.collision_buffer_zone)
        head_moment = pymunk.moment_for_circle(head_mass, 0, head_collision_radius)
        head_body = pymunk.Body(head_mass, head_moment)
        head_shape = pymunk.Circle(head_body, head_collision_radius)
        head_shape.friction = 0.7
        head_shape.density = 0.5
        # Head can collide with all limbs but not torso (directly connected)
        # No separate shoulders - triangle base is the shoulders
        head_category = self.collision_categories['head']
        head_mask = 0xFFFF & ~(self.collision_categories['torso'])
        head_shape.filter = pymunk.ShapeFilter(categories=head_category, mask=head_mask)
        self.space.add(head_body, head_shape)
        self.bodies['head'] = head_body
        
        # Position head above torso, touching it
        head_y = torso_center_y - scaled_torso/2 - scaled_head_radius
        head_body.position = (self.x, head_y)
        head_body.angle = 0
        
        # Connect head to torso - RIGID attachment (no leaning, fixed to torso)
        # Head attaches at bottom of head circle, torso attaches at top center (middle of shoulder base)
        head_anchor_local = (0, scaled_head_radius)  # Local to head (bottom of circle)
        torso_anchor_local = (0, -scaled_torso/2)  # Local to torso (top center of triangle)
        head_torso_joint = pymunk.PinJoint(head_body, torso_body, head_anchor_local, torso_anchor_local)
        
        # Use GearJoint to lock head rotation to torso rotation (rigid attachment)
        # Ratio of 1.0 means head rotates exactly with torso
        head_torso_gear = pymunk.GearJoint(head_body, torso_body, 0, 1.0)  # 1.0 ratio = same rotation
        # Make it very stiff
        head_torso_gear.error_bias = 0.0  # No error tolerance
        head_torso_gear.max_bias = 1000000.0  # Very high max bias for rigid connection
        
        # Also add a very strong distance spring to keep head fixed in position (backup rigidity)
        head_rest_distance = 0  # Pin joint, so rest distance is 0 (touching)
        head_torso_distance_spring = pymunk.DampedSpring(head_body, torso_body, 
                                                         head_anchor_local, torso_anchor_local,
                                                         head_rest_distance, 50000.0, 1000.0)  # Very strong spring
        
        self.space.add(head_torso_joint, head_torso_gear, head_torso_distance_spring)
        self.joints['head_torso'] = head_torso_joint
        self.constraints['head_torso_gear'] = head_torso_gear
        self.constraints['head_torso_distance_spring'] = head_torso_distance_spring
        
        # Arms attach directly to triangle's shoulder corners (base of triangle)
        # Left upper arm - connected to left shoulder corner of triangle (top left vertex)
        left_upper_arm = self._create_limb_segment('left_upper_arm', scaled_upper_arm, self.mass * 0.8)
        left_shoulder_local = (-triangle_width_top/2, triangle_base_top)  # Left shoulder corner of triangle
        left_shoulder_world = torso_body.local_to_world(left_shoulder_local)
        left_upper_arm.position = (left_shoulder_world.x - scaled_upper_arm/2, left_shoulder_world.y)
        left_upper_arm_initial_angle = math.radians(135)  # Down-left initially
        left_upper_arm.angle = left_upper_arm_initial_angle
        left_shoulder_joint = pymunk.PinJoint(left_upper_arm, torso_body, (-scaled_upper_arm/2, 0), left_shoulder_local)
        self.space.add(left_shoulder_joint)
        self.joints['left_shoulder'] = left_shoulder_joint
        
        # Rotation limits will be enforced manually in update method
        
        # Add distance constraint to prevent stretching
        left_shoulder_distance = pymunk.DampedSpring(left_upper_arm, torso_body,
                                                     (-scaled_upper_arm/2, 0), left_shoulder_local,
                                                     0, 2000.0, 100.0)
        self.space.add(left_shoulder_distance)
        self.constraints['left_shoulder_distance'] = left_shoulder_distance
        
        # Store initial angle for reference
        self.left_upper_arm_initial_angle = left_upper_arm_initial_angle
        
        # Right upper arm - connected to right shoulder corner of triangle (top right vertex)
        right_upper_arm = self._create_limb_segment('right_upper_arm', scaled_upper_arm, self.mass * 0.8)
        right_shoulder_local = (triangle_width_top/2, triangle_base_top)  # Right shoulder corner of triangle
        right_shoulder_world = torso_body.local_to_world(right_shoulder_local)
        right_upper_arm.position = (right_shoulder_world.x - scaled_upper_arm/2, right_shoulder_world.y)
        right_upper_arm_initial_angle = math.radians(45)  # Down-right initially
        right_upper_arm.angle = right_upper_arm_initial_angle
        right_shoulder_joint = pymunk.PinJoint(right_upper_arm, torso_body, (-scaled_upper_arm/2, 0), right_shoulder_local)
        self.space.add(right_shoulder_joint)
        self.joints['right_shoulder'] = right_shoulder_joint
        
        # Rotation limits will be enforced manually in update method
        
        # Add distance constraint
        right_shoulder_distance = pymunk.DampedSpring(right_upper_arm, torso_body,
                                                      (-scaled_upper_arm/2, 0), right_shoulder_local,
                                                      0, 2000.0, 100.0)
        self.space.add(right_shoulder_distance)
        self.constraints['right_shoulder_distance'] = right_shoulder_distance
        
        # Store initial angle for reference
        self.right_upper_arm_initial_angle = right_upper_arm_initial_angle
        
        # Left forearm - connected to end of left upper arm
        left_forearm = self._create_limb_segment('left_forearm', scaled_forearm, self.mass * 0.6)
        left_elbow_pos = left_upper_arm.local_to_world((scaled_upper_arm/2, 0))
        left_forearm.position = (left_elbow_pos.x - scaled_forearm/2, left_elbow_pos.y)
        left_forearm.angle = math.radians(135)  # Extends from upper arm
        left_elbow_joint = pymunk.PinJoint(left_forearm, left_upper_arm, (-scaled_forearm/2, 0), (scaled_upper_arm/2, 0))
        self.space.add(left_elbow_joint)
        self.joints['left_elbow'] = left_elbow_joint
        # Add distance constraint with max stretch limit
        max_elbow_distance = 0  # Pin joint, so rest length is 0, max stretch prevents over-stretching
        left_elbow_distance = pymunk.DampedSpring(left_forearm, left_upper_arm,
                                                  (-scaled_forearm/2, 0), (scaled_upper_arm/2, 0),
                                                  max_elbow_distance, 2000.0, 100.0)
        self.space.add(left_elbow_distance)
        self.constraints['left_elbow_distance'] = left_elbow_distance
        
        # Right forearm - connected to end of right upper arm
        right_forearm = self._create_limb_segment('right_forearm', scaled_forearm, self.mass * 0.6)
        right_elbow_pos = right_upper_arm.local_to_world((scaled_upper_arm/2, 0))
        right_forearm.position = (right_elbow_pos.x - scaled_forearm/2, right_elbow_pos.y)
        right_forearm.angle = math.radians(45)  # Extends from upper arm
        right_elbow_joint = pymunk.PinJoint(right_forearm, right_upper_arm, (-scaled_forearm/2, 0), (scaled_upper_arm/2, 0))
        self.space.add(right_elbow_joint)
        self.joints['right_elbow'] = right_elbow_joint
        # Add distance constraint
        right_elbow_distance = pymunk.DampedSpring(right_forearm, right_upper_arm,
                                                   (-scaled_forearm/2, 0), (scaled_upper_arm/2, 0),
                                                   0, 2000.0, 100.0)
        self.space.add(right_elbow_distance)
        self.constraints['right_elbow_distance'] = right_elbow_distance
        
        # Hips (hip connection point) - bottom apex of triangle
        # Both thighs attach to the same apex point (triangle bottom point)
        hip_apex_local = (0, triangle_apex_bottom)  # Triangle apex at bottom (hips)
        hip_apex_world = torso_body.local_to_world(hip_apex_local)
        
        # Left thigh - connected to triangle apex (bottom point)
        left_thigh = self._create_limb_segment('left_thigh', scaled_thigh, self.mass * 1.0)
        left_thigh_body = self.bodies['left_thigh']
        # Position thigh so it attaches at its top to the triangle apex
        left_thigh_body.position = (hip_apex_world.x - scaled_thigh/2, hip_apex_world.y)
        left_thigh_body.angle = math.pi / 2  # Down initially
        left_hip_joint = pymunk.PinJoint(torso_body, left_thigh_body, hip_apex_local, (-scaled_thigh/2, 0))
        self.space.add(left_hip_joint)
        self.joints['left_hip'] = left_hip_joint
        # Add distance constraint
        left_hip_distance = pymunk.DampedSpring(torso_body, left_thigh_body,
                                                hip_apex_local, (-scaled_thigh/2, 0),
                                                0, 2000.0, 100.0)
        self.space.add(left_hip_distance)
        self.constraints['left_hip_distance'] = left_hip_distance
        
        # Right thigh - connected to triangle apex (bottom point - same point as left)
        right_thigh = self._create_limb_segment('right_thigh', scaled_thigh, self.mass * 1.0)
        right_thigh_body = self.bodies['right_thigh']
        # Position thigh so it attaches at its top to the triangle apex
        right_thigh_body.position = (hip_apex_world.x - scaled_thigh/2, hip_apex_world.y)
        right_thigh_body.angle = math.pi / 2  # Down initially
        right_hip_joint = pymunk.PinJoint(torso_body, right_thigh_body, hip_apex_local, (-scaled_thigh/2, 0))
        self.space.add(right_hip_joint)
        self.joints['right_hip'] = right_hip_joint
        # Add distance constraint
        right_hip_distance = pymunk.DampedSpring(torso_body, right_thigh_body,
                                                 hip_apex_local, (-scaled_thigh/2, 0),
                                                 0, 2000.0, 100.0)
        self.space.add(right_hip_distance)
        self.constraints['right_hip_distance'] = right_hip_distance
        
        # Left shin - connected to end of left thigh
        left_shin = self._create_limb_segment('left_shin', scaled_shin, self.mass * 0.7)
        left_knee_pos = left_thigh_body.local_to_world((scaled_thigh/2, 0))
        left_shin.position = (left_knee_pos.x - scaled_shin/2, left_knee_pos.y)
        left_shin.angle = math.pi / 2  # Down initially
        left_knee_joint = pymunk.PinJoint(left_shin, left_thigh_body, (-scaled_shin/2, 0), (scaled_thigh/2, 0))
        self.space.add(left_knee_joint)
        self.joints['left_knee'] = left_knee_joint
        # Add distance constraint
        left_knee_distance = pymunk.DampedSpring(left_shin, left_thigh_body,
                                                 (-scaled_shin/2, 0), (scaled_thigh/2, 0),
                                                 0, 2000.0, 100.0)
        self.space.add(left_knee_distance)
        self.constraints['left_knee_distance'] = left_knee_distance
        
        # Right shin - connected to end of right thigh
        right_shin = self._create_limb_segment('right_shin', scaled_shin, self.mass * 0.7)
        right_knee_pos = right_thigh_body.local_to_world((scaled_thigh/2, 0))
        right_shin.position = (right_knee_pos.x - scaled_shin/2, right_knee_pos.y)
        right_shin.angle = math.pi / 2  # Down initially
        right_knee_joint = pymunk.PinJoint(right_shin, right_thigh_body, (-scaled_shin/2, 0), (scaled_thigh/2, 0))
        self.space.add(right_knee_joint)
        self.joints['right_knee'] = right_knee_joint
        # Add distance constraint
        right_knee_distance = pymunk.DampedSpring(right_shin, right_thigh_body,
                                                  (-scaled_shin/2, 0), (scaled_thigh/2, 0),
                                                  0, 2000.0, 100.0)
        self.space.add(right_knee_distance)
        self.constraints['right_knee_distance'] = right_knee_distance
        
        # Damping will be applied in update method
    
    def _create_limb_segment(self, name, length, mass):
        """Create a limb segment body with thickness and collision buffer zone for collision physics"""
        base_thickness = self._apply_scale(self.limb_thickness)
        # Add collision buffer zone to prevent limbs from crossing each other
        # (thickness is radius, so adding buffer increases diameter by 2*buffer, giving buffer on each side)
        collision_thickness = base_thickness + self._apply_scale(self.collision_buffer_zone)
        moment = pymunk.moment_for_segment(mass, (-length/2, 0), (length/2, 0), collision_thickness)
        body = pymunk.Body(mass, moment)
        shape = pymunk.Segment(body, (-length/2, 0), (length/2, 0), collision_thickness)
        shape.friction = 0.7  # Increased friction for better stability
        shape.density = 0.5  # Set density for collision response
        
        # Set up collision filtering: limbs can collide with each other but not with directly connected parts
        category = self.collision_categories.get(name, 0x1)
        
        # Determine what this limb should NOT collide with (directly connected parts)
        excluded_categories = 0
        if name == 'left_upper_arm':
            # Upper arms attach to torso triangle base (shoulders), so exclude torso and forearm
            excluded_categories = (self.collision_categories['torso'] | 
                                  self.collision_categories['left_forearm'])
        elif name == 'right_upper_arm':
            # Upper arms attach to torso triangle base (shoulders), so exclude torso and forearm
            excluded_categories = (self.collision_categories['torso'] | 
                                  self.collision_categories['right_forearm'])
        elif name == 'left_forearm':
            excluded_categories = self.collision_categories['left_upper_arm']
        elif name == 'right_forearm':
            excluded_categories = self.collision_categories['right_upper_arm']
        elif name == 'left_thigh':
            excluded_categories = (self.collision_categories['torso'] | 
                                  self.collision_categories['left_shin'])
        elif name == 'right_thigh':
            excluded_categories = (self.collision_categories['torso'] | 
                                  self.collision_categories['right_shin'])
        elif name == 'left_shin':
            excluded_categories = self.collision_categories['left_thigh']
        elif name == 'right_shin':
            excluded_categories = self.collision_categories['right_thigh']
        
        # Can collide with everything except directly connected parts and itself
        mask = 0xFFFF & ~excluded_categories & ~category
        
        shape.filter = pymunk.ShapeFilter(categories=category, mask=mask)
        self.space.add(body, shape)
        self.bodies[name] = body
        self.limb_lengths[name] = length  # Store length for easy access
        return body
    
    def _enforce_rigid_head_attachment(self):
        """
        Keep head rigidly attached to torso - match angle exactly
        Head rotates with torso, no independent leaning
        """
        head_body = self.bodies.get('head')
        torso_body = self.bodies.get('torso')
        
        if not head_body or not torso_body:
            return
        
        try:
            # Make head angle match torso angle exactly (rigid attachment)
            # This ensures head never leans independently
            head_body.angle = torso_body.angle
            
            # Also enforce position to keep head attached (rigid position constraint)
            # Get anchor points
            scaled_head_radius = self._apply_scale(self.head_radius)
            scaled_torso = self._apply_scale(self.torso_length)
            
            head_anchor_local = (0, scaled_head_radius)  # Bottom of head circle (attachment point)
            torso_anchor_local = (0, -scaled_torso/2)  # Top of torso triangle (attachment point)
            
            # Calculate where head center should be based on torso position and angle
            # Head attaches at its bottom, so head center is above the attachment point
            torso_anchor_world = torso_body.local_to_world(torso_anchor_local)
            torso_anchor_x = float(torso_anchor_world.x) if hasattr(torso_anchor_world, 'x') else float(torso_anchor_world[0])
            torso_anchor_y = float(torso_anchor_world.y) if hasattr(torso_anchor_world, 'y') else float(torso_anchor_world[1])
            
            # Head center is head_radius distance above the attachment point
            # Since head rotates with torso, calculate offset in world space
            # From attachment point, head center is up (negative local y) relative to torso
            # Convert to world space accounting for torso rotation
            head_offset_x = -scaled_head_radius * math.sin(torso_body.angle)
            head_offset_y = -scaled_head_radius * math.cos(torso_body.angle)
            
            expected_head_x = torso_anchor_x + head_offset_x
            expected_head_y = torso_anchor_y + head_offset_y
            
            # Apply strong restoring force if head position drifts
            current_pos = head_body.position
            current_x = float(current_pos.x) if hasattr(current_pos, 'x') else float(current_pos[0])
            current_y = float(current_pos.y) if hasattr(current_pos, 'y') else float(current_pos[1])
            
            dx = expected_head_x - current_x
            dy = expected_head_y - current_y
            
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > 0.1:  # Only apply if position has drifted
                # Very strong restoring force to keep head rigidly attached
                restore_force_x = dx * 100000.0
                restore_force_y = dy * 100000.0
                max_force = 1000000.0
                restore_force_x = max(-max_force, min(max_force, restore_force_x))
                restore_force_y = max(-max_force, min(max_force, restore_force_y))
                head_body.apply_force_at_local_point((restore_force_x, restore_force_y), (0, 0))
                
                # Also set position directly if drift is too large (hard clamp)
                if distance > 2.0:
                    head_body.position = (expected_head_x, expected_head_y)
                    head_body.velocity = (0, 0)  # Stop any velocity
            
            # Ensure head angular velocity matches torso (rigid rotation)
            head_body.angular_velocity = torso_body.angular_velocity
            
        except (AttributeError, TypeError, ValueError, OverflowError, ZeroDivisionError):
            pass  # Skip if calculation fails
    
    
    def _enforce_max_distances(self):
        """
        Enforce maximum distances between connected parts to prevent over-stretching
        This prevents limbs from becoming "silly string" and flying off
        Uses the actual joint positions calculated from bodies
        """
        try:
            positions = self.get_joint_positions()
        except:
            return  # Skip if we can't get positions
        
        # Head is now rigidly attached - no distance enforcement needed
        
        # Define maximum allowed distances between joint points
        # Format: (pos1_key, pos2_key, max_distance_factor, base_length)
        distance_limits = [
            # Head to neck (should stay VERY close - critical for head staying on)
            # Head distance is handled separately in _enforce_head_distance()
            # Arms (shoulders are part of triangle base now)
            ('left_shoulder', 'left_elbow', 1.3, self._apply_scale(self.upper_arm_length)),
            ('left_elbow', 'left_wrist', 1.3, self._apply_scale(self.forearm_length)),
            ('right_shoulder', 'right_elbow', 1.3, self._apply_scale(self.upper_arm_length)),
            ('right_elbow', 'right_wrist', 1.3, self._apply_scale(self.forearm_length)),
            # Legs (both attach to triangle apex)
            ('left_hip', 'left_knee', 1.3, self._apply_scale(self.thigh_length)),
            ('left_knee', 'left_ankle', 1.3, self._apply_scale(self.shin_length)),
            ('right_hip', 'right_knee', 1.3, self._apply_scale(self.thigh_length)),
            ('right_knee', 'right_ankle', 1.3, self._apply_scale(self.shin_length)),
        ]
        
        for pos1_key, pos2_key, max_factor, base_length in distance_limits:
            pos1 = positions.get(pos1_key)
            pos2 = positions.get(pos2_key)
            
            if not pos1 or not pos2:
                continue
            
            try:
                # Extract coordinates safely
                if isinstance(pos1, (tuple, list)) and isinstance(pos2, (tuple, list)):
                    x1, y1 = float(pos1[0]), float(pos1[1])
                    x2, y2 = float(pos2[0]), float(pos2[1])
                else:
                    continue
                
                # Calculate current distance
                dx = x1 - x2
                dy = y1 - y2
                
                # Clamp to prevent overflow
                dx = max(-5000, min(5000, dx))
                dy = max(-5000, min(5000, dy))
                
                current_distance_sq = dx * dx + dy * dy
                if current_distance_sq > 1e8:  # Overflow protection
                    continue
                
                current_distance = math.sqrt(current_distance_sq)
                max_distance = base_length * max_factor
                
                # If distance exceeds maximum, apply restoring force to the bodies
                if current_distance > max_distance:
                    # Find which bodies these positions belong to
                    body1 = None
                    body2 = None
                    
                    # Map positions to bodies
                    if pos1_key in ['head_center']:
                        body1 = self.bodies.get('head')
                    elif pos1_key in ['neck', 'waist', 'shoulder_center', 'left_shoulder', 'right_shoulder']:
                        # Shoulders and neck/waist are part of torso triangle
                        body1 = self.bodies.get('torso')
                    elif pos1_key in ['left_elbow', 'left_wrist']:
                        if 'elbow' in pos1_key:
                            body1 = self.bodies.get('left_upper_arm')
                        elif 'wrist' in pos1_key:
                            body1 = self.bodies.get('left_forearm')
                    elif pos1_key in ['right_elbow', 'right_wrist']:
                        if 'elbow' in pos1_key:
                            body1 = self.bodies.get('right_upper_arm')
                        elif 'wrist' in pos1_key:
                            body1 = self.bodies.get('right_forearm')
                    elif pos1_key in ['left_hip', 'left_knee', 'left_ankle']:
                        if 'hip' in pos1_key:
                            body1 = self.bodies.get('left_thigh')
                        elif 'knee' in pos1_key:
                            body1 = self.bodies.get('left_shin')
                        elif 'ankle' in pos1_key:
                            body1 = self.bodies.get('left_shin')
                    elif pos1_key in ['right_hip', 'right_knee', 'right_ankle']:
                        if 'hip' in pos1_key:
                            body1 = self.bodies.get('right_thigh')
                        elif 'knee' in pos1_key:
                            body1 = self.bodies.get('right_shin')
                        elif 'ankle' in pos1_key:
                            body1 = self.bodies.get('right_shin')
                    
                    # Same for pos2_key
                    if pos2_key in ['neck', 'waist', 'shoulder_center', 'left_shoulder', 'right_shoulder', 'left_hip', 'right_hip']:
                        # Shoulders, neck, waist, and hips are part of torso triangle
                        body2 = self.bodies.get('torso')
                    elif pos2_key in ['left_elbow', 'left_wrist']:
                        if 'elbow' in pos2_key:
                            body2 = self.bodies.get('left_upper_arm')
                        else:
                            body2 = self.bodies.get('left_forearm')
                    elif pos2_key in ['right_elbow', 'right_wrist']:
                        if 'elbow' in pos2_key:
                            body2 = self.bodies.get('right_upper_arm')
                        else:
                            body2 = self.bodies.get('right_forearm')
                    elif pos2_key in ['left_knee', 'left_ankle']:
                        if 'knee' in pos2_key:
                            body2 = self.bodies.get('left_thigh')
                        elif 'ankle' in pos2_key:
                            body2 = self.bodies.get('left_shin')
                    elif pos2_key in ['right_knee', 'right_ankle']:
                        if 'knee' in pos2_key:
                            body2 = self.bodies.get('right_thigh')
                        elif 'ankle' in pos2_key:
                            body2 = self.bodies.get('right_shin')
                    
                    # If we found both bodies, apply restoring forces
                    if body1 and body2 and body1 != body2:
                        # Calculate direction from pos2 to pos1
                        if current_distance > 0.1:
                            direction_x = dx / current_distance
                            direction_y = dy / current_distance
                            
                            # Calculate excess distance
                            excess_distance = current_distance - max_distance
                            
                            # Apply strong restoring force
                            restore_force_strength = 10000.0  # Very strong to prevent stretching
                            force_x = direction_x * excess_distance * restore_force_strength
                            force_y = direction_y * excess_distance * restore_force_strength
                            
                            # Clamp forces
                            max_force = 100000.0
                            force_x = max(-max_force, min(max_force, force_x))
                            force_y = max(-max_force, min(max_force, force_y))
                            
                            # Apply forces at center of bodies (simpler and more stable)
                            body1.apply_force_at_local_point((force_x, force_y), (0, 0))
                            body2.apply_force_at_local_point((-force_x, -force_y), (0, 0))
            except (AttributeError, TypeError, ValueError, OverflowError, ZeroDivisionError) as e:
                # Skip this connection if calculation fails
                continue
    
    def _enforce_shoulder_rotation_limits(self):
        """Enforce rotation limits on shoulder joints to prevent arms wrapping around body"""
        torso_body = self.bodies.get('torso')
        left_arm_body = self.bodies.get('left_upper_arm')
        right_arm_body = self.bodies.get('right_upper_arm')
        
        if not torso_body:
            return
        
        try:
            torso_angle = torso_body.angle
            
            # Left shoulder: limit relative angle between left arm and torso
            if left_arm_body and hasattr(self, 'left_upper_arm_initial_angle'):
                # Calculate relative angle (arm angle - torso angle)
                arm_angle = left_arm_body.angle
                relative_angle = arm_angle - torso_angle
                
                # Normalize relative angle to [-π, π] range
                while relative_angle > math.pi:
                    relative_angle -= 2 * math.pi
                while relative_angle < -math.pi:
                    relative_angle += 2 * math.pi
                
                # Initial relative angle: 135° - 90° = 45° (45 degrees relative to torso)
                initial_relative = self.left_upper_arm_initial_angle - self.torso_initial_angle
                
                # Allow ±90 degrees from initial relative angle
                # This prevents arm from wrapping to the right side of body
                min_relative = initial_relative - self.left_shoulder_max_rotation
                max_relative = initial_relative + self.left_shoulder_max_rotation
                
                # Clamp relative angle if it exceeds limits
                if relative_angle > max_relative:
                    # Exceeded max - clamp arm angle
                    left_arm_body.angle = torso_angle + max_relative
                    left_arm_body.angular_velocity *= 0.5  # Dampen angular velocity
                elif relative_angle < min_relative:
                    # Exceeded min - clamp arm angle
                    left_arm_body.angle = torso_angle + min_relative
                    left_arm_body.angular_velocity *= 0.5  # Dampen angular velocity
                
                # Apply restoring torque if near limits
                if abs(relative_angle - max_relative) < math.radians(10) or abs(relative_angle - min_relative) < math.radians(10):
                    excess = max(0, abs(relative_angle) - abs(max_relative))
                    if relative_angle > 0:
                        restore_torque = -excess * 50000.0
                    else:
                        restore_torque = excess * 50000.0
                    left_arm_body.torque += restore_torque
            
            # Right shoulder: limit relative angle between right arm and torso
            if right_arm_body and hasattr(self, 'right_upper_arm_initial_angle'):
                # Calculate relative angle (arm angle - torso angle)
                arm_angle = right_arm_body.angle
                relative_angle = arm_angle - torso_angle
                
                # Normalize relative angle to [-π, π] range
                while relative_angle > math.pi:
                    relative_angle -= 2 * math.pi
                while relative_angle < -math.pi:
                    relative_angle += 2 * math.pi
                
                # Initial relative angle: 45° - 90° = -45° (-45 degrees relative to torso)
                initial_relative = self.right_upper_arm_initial_angle - self.torso_initial_angle
                
                # Allow ±90 degrees from initial relative angle
                # This prevents arm from wrapping to the left side of body
                min_relative = initial_relative - self.right_shoulder_max_rotation
                max_relative = initial_relative + self.right_shoulder_max_rotation
                
                # Clamp relative angle if it exceeds limits
                if relative_angle > max_relative:
                    # Exceeded max - clamp arm angle
                    right_arm_body.angle = torso_angle + max_relative
                    right_arm_body.angular_velocity *= 0.5  # Dampen angular velocity
                elif relative_angle < min_relative:
                    # Exceeded min - clamp arm angle
                    right_arm_body.angle = torso_angle + min_relative
                    right_arm_body.angular_velocity *= 0.5  # Dampen angular velocity
                
                # Apply restoring torque if near limits
                if abs(relative_angle - max_relative) < math.radians(10) or abs(relative_angle - min_relative) < math.radians(10):
                    excess = max(0, abs(relative_angle) - abs(max_relative))
                    if relative_angle > 0:
                        restore_torque = -excess * 50000.0
                    else:
                        restore_torque = excess * 50000.0
                    right_arm_body.torque += restore_torque
                    
        except (AttributeError, TypeError, ValueError, OverflowError):
            pass  # Skip if calculation fails
    
    def _enforce_torso_rotation_limit(self):
        """Enforce maximum rotation limit on torso (±100 degrees from initial angle)"""
        torso_body = self.bodies.get('torso')
        if not torso_body:
            return
        
        try:
            # Calculate current angle relative to initial angle
            current_angle = torso_body.angle
            angle_diff = current_angle - self.torso_initial_angle
            
            # Normalize angle difference to [-π, π] range for comparison
            angle_diff_normalized = angle_diff
            while angle_diff_normalized > math.pi:
                angle_diff_normalized -= 2 * math.pi
            while angle_diff_normalized < -math.pi:
                angle_diff_normalized += 2 * math.pi
            
            # Calculate allowed limits in absolute angle space
            max_allowed_angle = self.torso_initial_angle + self.torso_max_rotation
            min_allowed_angle = self.torso_initial_angle - self.torso_max_rotation
            
            # HARD CLAMP: Always clamp angle if it exceeds or equals limits (before any other calculations)
            # Use >= to catch angles exactly at the limit and clamp them to be safe
            angle_was_clamped = False
            if abs(angle_diff_normalized) >= self.torso_max_rotation:
                # Angle at or exceeds limit - clamp it immediately to exactly the limit
                if angle_diff_normalized > 0:
                    # Clockwise at/exceeds max - clamp to exactly max
                    torso_body.angle = max_allowed_angle
                    angle_diff_normalized = self.torso_max_rotation
                elif angle_diff_normalized < 0:
                    # Counter-clockwise at/exceeds min - clamp to exactly min
                    torso_body.angle = min_allowed_angle
                    angle_diff_normalized = -self.torso_max_rotation
                else:
                    # Exactly 0 or exactly at limit - keep it at limit (shouldn't happen but safety)
                    if angle_diff_normalized >= 0:
                        torso_body.angle = max_allowed_angle
                        angle_diff_normalized = self.torso_max_rotation
                    else:
                        torso_body.angle = min_allowed_angle
                        angle_diff_normalized = -self.torso_max_rotation
                angle_was_clamped = True
                
                # Also clamp angular velocity to zero when angle is clamped
                torso_body.angular_velocity = 0.0
            
            # Check if rotation is at or near the limit (within small tolerance)
            tolerance = 0.001  # Small tolerance in radians
            if abs(angle_diff_normalized) >= (self.torso_max_rotation - tolerance):
                # Calculate excess rotation if any
                excess_rotation = abs(angle_diff_normalized) - self.torso_max_rotation
                if excess_rotation < 0:
                    excess_rotation = 0
                
                # Apply very strong restoring torque to prevent exceeding limit
                if angle_diff_normalized > 0:
                    # Rotated too far clockwise, apply counter-clockwise torque
                    restore_torque = -(excess_rotation + 0.01) * 200000.0  # Very strong torque
                else:
                    # Rotated too far counter-clockwise, apply clockwise torque
                    restore_torque = (excess_rotation + 0.01) * 200000.0
                
                # Clamp torque to prevent overflow
                max_torque = 500000.0
                restore_torque = max(-max_torque, min(max_torque, restore_torque))
                
                # Apply restoring torque
                torso_body.torque += restore_torque
                
                # Aggressively control angular velocity when at limits
                if abs(torso_body.angular_velocity) > 0.01:
                    # If rotating in the direction that would push past limit, stop it
                    if angle_diff_normalized >= (self.torso_max_rotation - tolerance) and torso_body.angular_velocity > 0:
                        # At/near max limit rotating clockwise - stop or reverse
                        torso_body.angular_velocity = -abs(torso_body.angular_velocity) * 0.3  # Reverse and dampen
                    elif angle_diff_normalized <= -(self.torso_max_rotation - tolerance) and torso_body.angular_velocity < 0:
                        # At/near min limit rotating counter-clockwise - stop or reverse
                        torso_body.angular_velocity = abs(torso_body.angular_velocity) * 0.3  # Reverse and dampen
                    else:
                        # Very aggressive damping when at limits
                        torso_body.angular_velocity *= 0.2  # Very aggressive damping
                
                # Final safety clamp - double-check angle hasn't exceeded limit (use >= for safety)
                final_angle = torso_body.angle
                final_angle_diff = final_angle - self.torso_initial_angle
                # Normalize final angle diff
                while final_angle_diff > math.pi:
                    final_angle_diff -= 2 * math.pi
                while final_angle_diff < -math.pi:
                    final_angle_diff += 2 * math.pi
                
                # Hard clamp if at or exceeded limit (use >= to ensure it never exceeds)
                if final_angle_diff >= self.torso_max_rotation:
                    torso_body.angle = max_allowed_angle
                    torso_body.angular_velocity = 0.0
                    # Clear any accumulated torque that might push past limit
                    torso_body.torque = 0.0
                elif final_angle_diff <= -self.torso_max_rotation:
                    torso_body.angle = min_allowed_angle
                    torso_body.angular_velocity = 0.0
                    # Clear any accumulated torque that might push past limit
                    torso_body.torque = 0.0
                    
        except (AttributeError, TypeError, ValueError, OverflowError):
            # Skip if calculation fails
            pass
    
    def _maintain_rest_pose(self, dt):
        """
        Apply gentle restorative forces/torques to maintain a natural upright pose
        Prevents drooping by bringing limbs back toward rest angles
        """
        if not self.rest_angles:
            return
        
        for name, body in self.bodies.items():
            if name == 'center_pivot':  # Skip static pivot
                continue
            
            # Skip head - it's rigidly attached, angle is enforced separately
            if name == 'head':
                continue
            
            rest_angle = self.rest_angles.get(name)
            if rest_angle is None:
                continue
            
            try:
                current_angle = body.angle
                angle_diff = current_angle - rest_angle
                
                # Normalize angle difference to [-π, π] range
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Apply restorative torque toward rest angle
                # Only apply if angle is significantly off (to allow dynamic movement)
                if abs(angle_diff) > math.radians(5):  # 5 degree threshold
                    # Gentle restorative torque - stronger as deviation increases
                    restore_torque = -angle_diff * self.rest_pose_strength
                    
                    # Scale torque based on body type - lighter for force-applied bodies
                    force_applied_bodies = ['left_upper_arm', 'right_upper_arm', 'left_thigh', 'right_thigh']
                    if name in force_applied_bodies:
                        restore_torque *= 0.5  # Half strength for bodies with applied forces
                    
                    # Clamp torque to prevent overflow
                    max_torque = 10000.0
                    restore_torque = max(-max_torque, min(max_torque, restore_torque))
                    body.torque += restore_torque
                
                # Skip head - it's rigidly attached to torso, angle/position handled separately
                if name == 'head':
                    continue  # Head is rigidly attached, no independent rest pose
                
                # For torso: apply gentle upward force at top to prevent forward lean
                if name == 'torso':
                    torso_body = body
                    # Apply upward force at top of torso to counteract forward lean
                    torso_top_local = (0, -self._apply_scale(self.torso_length / 2))
                    upward_force_y = -self.rest_pose_strength * 0.2  # Upward force
                    # Only apply if torso is leaning forward significantly
                    angle_from_vertical = abs(angle_diff)
                    if angle_from_vertical > math.radians(10):  # More than 10 degrees from vertical
                        lean_correction = self.rest_pose_strength * 0.3 * (angle_from_vertical / math.pi)
                        # Apply force perpendicular to torso to straighten it
                        correction_force_x = math.sin(angle_diff) * lean_correction
                        correction_force_y = -abs(math.cos(angle_diff)) * lean_correction  # Upward
                        torso_body.apply_force_at_local_point((correction_force_x, correction_force_y), torso_top_local)
                
            except (AttributeError, TypeError, ValueError, OverflowError):
                pass  # Skip if calculation fails
    
    def _apply_damping(self, dt):
        """Apply damping to all bodies - less damping for bodies with applied forces"""
        force_applied_bodies = ['left_upper_arm', 'right_upper_arm', 'left_thigh', 'right_thigh']
        for name, body in self.bodies.items():
            if name == 'center_pivot':  # Skip static pivot body
                continue
            # Apply less damping to bodies with forces applied so they respond better
            if name in force_applied_bodies:
                # Lighter damping for force-applied bodies (0.98 vs 0.96)
                body.velocity = body.velocity * (0.98 ** dt)
                body.angular_velocity = body.angular_velocity * (0.98 ** dt)
            else:
                # Standard damping for other bodies
                body.velocity = body.velocity * (self.damping ** dt)
                body.angular_velocity = body.angular_velocity * (self.damping ** dt)
    
    def _apply_normalizing_forces(self, dt):
        """
        Apply normalizing forces to help stabilize the stick figure
        - Reduces excessive velocity
        - Helps restore center position for torso
        - Adds stability to all parts
        """
        # Get center pivot position
        center_pivot = self.bodies.get('center_pivot')
        if center_pivot is None:
            return
        
        center_x, center_y = center_pivot.position
        
        for name, body in self.bodies.items():
            if name == 'center_pivot':  # Skip static pivot body
                continue
            
            # Normalize velocities (reduce excessive movement)
            # Extract and validate velocity components to prevent overflow
            vel_x = float(body.velocity.x)
            vel_y = float(body.velocity.y)
            
            # Check for invalid values (NaN, inf) and clamp to reasonable ranges
            max_vel_component = 5000.0  # Maximum velocity component to prevent overflow
            if math.isnan(vel_x) or math.isinf(vel_x) or abs(vel_x) > max_vel_component:
                vel_x = 0.0
                body.velocity = (0.0, body.velocity.y)
            else:
                vel_x = max(-max_vel_component, min(max_vel_component, vel_x))
            
            if math.isnan(vel_y) or math.isinf(vel_y) or abs(vel_y) > max_vel_component:
                vel_y = 0.0
                body.velocity = (body.velocity.x, 0.0)
            else:
                vel_y = max(-max_vel_component, min(max_vel_component, vel_y))
            
            # Calculate velocity magnitude with overflow protection
            try:
                vel_x_sq = vel_x * vel_x
                vel_y_sq = vel_y * vel_y
                # Check for overflow before sqrt - use squared max_velocity for comparison
                max_vel_sq = self.max_velocity * self.max_velocity * 4  # Allow some headroom
                if vel_x_sq > max_vel_sq or vel_y_sq > max_vel_sq:
                    vel_magnitude = self.max_velocity * 2  # Force normalization
                else:
                    vel_magnitude = math.sqrt(vel_x_sq + vel_y_sq)
            except (OverflowError, ValueError):
                vel_magnitude = self.max_velocity * 2  # Force normalization on error
            
            # Only apply velocity damping to bodies that don't have forces directly applied
            # Bodies with applied forces (upper arms/thighs) should respond more freely
            force_applied_bodies = ['left_upper_arm', 'right_upper_arm', 'left_thigh', 'right_thigh']
            apply_velocity_damping = name not in force_applied_bodies
            
            if vel_magnitude > self.max_velocity and apply_velocity_damping:
                # Apply force opposite to velocity to reduce speed (only for non-force-applied bodies)
                damping_force_scale = (vel_magnitude - self.max_velocity) / self.max_velocity
                damping_force_x = -vel_x * self.normalizing_force_strength * damping_force_scale
                damping_force_y = -vel_y * self.normalizing_force_strength * damping_force_scale
                # Clamp forces to prevent overflow
                max_force = 50000.0
                damping_force_x = max(-max_force, min(max_force, damping_force_x))
                damping_force_y = max(-max_force, min(max_force, damping_force_y))
                body.apply_force_at_local_point((damping_force_x, damping_force_y), (0, 0))
            elif name in force_applied_bodies and vel_magnitude > self.max_velocity * 2.0:
                # For force-applied bodies, only damp if velocity gets EXTREMELY high (2x max)
                # This prevents explosion while still allowing forces to work
                damping_force_scale = (vel_magnitude - self.max_velocity * 2.0) / (self.max_velocity * 2.0)
                damping_force_x = -vel_x * self.normalizing_force_strength * damping_force_scale * 0.3  # Much weaker
                damping_force_y = -vel_y * self.normalizing_force_strength * damping_force_scale * 0.3  # Much weaker
                max_force = 50000.0
                damping_force_x = max(-max_force, min(max_force, damping_force_x))
                damping_force_y = max(-max_force, min(max_force, damping_force_y))
                body.apply_force_at_local_point((damping_force_x, damping_force_y), (0, 0))
            
            # Normalize angular velocity (reduce excessive rotation)
            ang_vel = body.angular_velocity
            # Check for invalid angular velocity
            if math.isnan(ang_vel) or math.isinf(ang_vel):
                ang_vel = 0.0
                body.angular_velocity = 0.0
            else:
                ang_vel = float(ang_vel)
                # Clamp angular velocity
                max_ang_vel_abs = 100.0  # Maximum absolute angular velocity
                if abs(ang_vel) > max_ang_vel_abs:
                    ang_vel = max_ang_vel_abs if ang_vel > 0 else -max_ang_vel_abs
                    body.angular_velocity = ang_vel
            
            if abs(ang_vel) > self.max_angular_velocity:
                damping_torque = -ang_vel * self.normalizing_force_strength * 10.0
                # Clamp torque to prevent overflow
                max_torque = 50000.0
                damping_torque = max(-max_torque, min(max_torque, damping_torque))
                body.torque += damping_torque
            
            # For torso: apply restoring force to keep it centered
            if name == 'torso':
                try:
                    # Calculate distance from center with overflow protection
                    dx = float(body.position.x) - float(center_x)
                    dy = float(body.position.y) - float(center_y)
                    
                    # Clamp position differences to prevent overflow
                    max_offset = 5000.0
                    dx = max(-max_offset, min(max_offset, dx))
                    dy = max(-max_offset, min(max_offset, dy))
                    
                    # Calculate distance with overflow protection
                    dx_sq = dx * dx
                    dy_sq = dy * dy
                    max_distance_sq = 1000000.0  # Max distance squared (1000 pixels)
                    if dx_sq > max_distance_sq or dy_sq > max_distance_sq:
                        distance = 1000.0  # Force restoration
                    else:
                        distance = math.sqrt(dx_sq + dy_sq)
                    
                    if distance > 5.0:  # Only apply if significantly off-center
                        # Apply restoring force toward center
                        restore_force_x = -dx * self.center_restore_force
                        restore_force_y = -dy * self.center_restore_force
                        # Clamp forces to prevent overflow
                        max_restore_force = 50000.0
                        restore_force_x = max(-max_restore_force, min(max_restore_force, restore_force_x))
                        restore_force_y = max(-max_restore_force, min(max_restore_force, restore_force_y))
                        body.apply_force_at_local_point((restore_force_x, restore_force_y), (0, 0))
                except (OverflowError, ValueError, TypeError, AttributeError):
                    # If calculation fails, skip restoration for this frame
                    pass
            
            # Apply general stability forces (reduces excessive movement)
            # BUT: Skip bodies that have forces directly applied (upper arms and thighs)
            # so they can respond freely to the vector forces without counteraction
            force_applied_bodies = ['left_upper_arm', 'right_upper_arm', 'left_thigh', 'right_thigh']
            if name not in force_applied_bodies:
                # Only apply stability forces to bodies that don't have direct forces applied
                # Use the validated/clamped velocities
                stability_force_x = -vel_x * self.normalizing_force_strength * 0.05  # Reduced from 0.1
                stability_force_y = -vel_y * self.normalizing_force_strength * 0.05  # Reduced from 0.1
                # Clamp stability forces to prevent overflow
                max_stability_force = 10000.0
                stability_force_x = max(-max_stability_force, min(max_stability_force, stability_force_x))
                stability_force_y = max(-max_stability_force, min(max_stability_force, stability_force_y))
                body.apply_force_at_local_point((stability_force_x, stability_force_y), (0, 0))
    
    def _set_initial_pose(self):
        """Set initial pose - positions should already be set, just ensure angles are correct"""
        # Bodies should already be positioned correctly during construction
        # Just ensure angles are set for a natural pose
        
        # Store rest angles for rest pose maintenance (upright, natural pose)
        # Torso: vertical (90 degrees)
        # Head: same as torso (rigidly attached, no independent angle)
        self.rest_angles['torso'] = math.pi / 2  # 90 degrees (vertical)
        self.rest_angles['head'] = math.pi / 2  # Head matches torso angle (rigid attachment)
        
        # Arms: slightly outward and down (more upright than drooping)
        self.rest_angles['left_upper_arm'] = math.radians(120)  # More outward, less drooped
        self.rest_angles['right_upper_arm'] = math.radians(60)  # More outward, less drooped
        self.rest_angles['left_forearm'] = math.radians(150)  # Extend from upper arm
        self.rest_angles['right_forearm'] = math.radians(30)  # Extend from upper arm
        
        # Legs: mostly vertical, slightly splayed
        self.rest_angles['left_thigh'] = math.radians(95)  # Slightly outward
        self.rest_angles['right_thigh'] = math.radians(85)  # Slightly outward
        self.rest_angles['left_shin'] = math.radians(95)  # Continue from thigh
        self.rest_angles['right_shin'] = math.radians(85)  # Continue from thigh
        
        # Apply initial angles
        left_arm_body = self.bodies.get('left_upper_arm')
        if left_arm_body:
            left_arm_body.angle = self.rest_angles['left_upper_arm']
        
        right_arm_body = self.bodies.get('right_upper_arm')
        if right_arm_body:
            right_arm_body.angle = self.rest_angles['right_upper_arm']
        
        # Forearms extend from upper arms
        left_forearm_body = self.bodies.get('left_forearm')
        if left_forearm_body:
            left_forearm_body.angle = self.rest_angles['left_forearm']
        
        right_forearm_body = self.bodies.get('right_forearm')
        if right_forearm_body:
            right_forearm_body.angle = self.rest_angles['right_forearm']
        
        # Legs already positioned, angles already set during construction
        left_thigh_body = self.bodies.get('left_thigh')
        if left_thigh_body:
            left_thigh_body.angle = self.rest_angles['left_thigh']
        
        right_thigh_body = self.bodies.get('right_thigh')
        if right_thigh_body:
            right_thigh_body.angle = self.rest_angles['right_thigh']
        
        left_shin_body = self.bodies.get('left_shin')
        if left_shin_body:
            left_shin_body.angle = self.rest_angles['left_shin']
        
        right_shin_body = self.bodies.get('right_shin')
        if right_shin_body:
            right_shin_body.angle = self.rest_angles['right_shin']
    
    def _apply_scale(self, value):
        """Apply scale to a value"""
        return value * self.scale
    
    def set_position(self, x, y):
        """Set the position of the stick figure (moves the torso/center)"""
        self.x = x
        self.y = y
        # Update torso position (will cascade through joints)
        torso_body = self.bodies['torso']
        current_torso_y = torso_body.position.y
        torso_body.position = (x, current_torso_y)
    
    def set_scale(self, scale):
        """Set the scale of the stick figure (requires rebuilding)"""
        self.scale = scale
        # Would need to rebuild physics structure - for now just update visual scale
    
    def set_color(self, color):
        """Set the color of the stick figure"""
        self.color = color
    
    def set_show_accelerations(self, show):
        """Enable or disable acceleration visualization"""
        self.show_accelerations = show
    
    def set_show_torso_rotation(self, show):
        """Enable or disable torso rotation visualization"""
        self.show_torso_rotation = show
    
    def set_force(self, limb, force):
        """
        Set target force for a limb (for sensor integration)
        Forces are applied to upper arms and thighs
        
        Args:
            limb: 'left_upper_arm', 'right_upper_arm', 'left_upper_leg', 'right_upper_leg'
            force: Tuple of (fx, fy, torque) in physics units
        """
        if limb == 'left_upper_arm':
            self.left_upper_arm_target_force = force
        elif limb == 'right_upper_arm':
            self.right_upper_arm_target_force = force
        elif limb == 'left_upper_leg':
            self.left_upper_leg_target_force = force
        elif limb == 'right_upper_leg':
            self.right_upper_leg_target_force = force
    
    def set_force_from_sensor(self, limb, accel_xyz, gyro_xyz=None):
        """
        Set force from sensor data (accelerometer + optional gyroscope)
        
        Args:
            limb: 'left_upper_arm', 'right_upper_arm', 'left_upper_leg', 'right_upper_leg'
            accel_xyz: (ax, ay, az) acceleration in m/s^2 or sensor units
            gyro_xyz: (gx, gy, gz) angular velocity in rad/s (optional, used for torque)
        """
        # Convert accelerometer data to forces
        fx = accel_xyz[0] * self.force_scale
        fy = accel_xyz[1] * self.force_scale
        torque = 0.0
        
        # Convert gyroscope data to torque if provided
        if gyro_xyz is not None:
            # Use z-component for 2D rotation (around z-axis)
            torque = gyro_xyz[2] * self.force_scale * 0.1
        
        self.set_force(limb, (fx, fy, torque))
    
    def _generate_random_forces(self):
        """Generate random target forces for testing"""
        current_time = time.time() * 1000
        
        if current_time - self.last_random_update > self.random_update_interval:
            if random.random() < self.random_force_change_rate:
                self.left_upper_arm_target_force = (
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength * 0.3, self.random_force_strength * 0.3)
                )
            
            if random.random() < self.random_force_change_rate:
                self.right_upper_arm_target_force = (
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength * 0.3, self.random_force_strength * 0.3)
                )
            
            if random.random() < self.random_force_change_rate:
                self.left_upper_leg_target_force = (
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength * 0.3, self.random_force_strength * 0.3)
                )
            
            if random.random() < self.random_force_change_rate:
                self.right_upper_leg_target_force = (
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength, self.random_force_strength),
                    random.uniform(-self.random_force_strength * 0.3, self.random_force_strength * 0.3)
                )
            
            self.last_random_update = current_time
    
    def _lerp_tuple(self, a, b, t):
        """Linear interpolation between two tuples"""
        return tuple(a[i] + (b[i] - a[i]) * t for i in range(len(a)))
    
    def _smooth_forces(self, dt):
        """Smoothly interpolate current forces toward target forces"""
        interp_factor = 1.0 - math.exp(-self.force_interpolation_speed * dt)
        
        self.left_upper_arm_force = self._lerp_tuple(
            self.left_upper_arm_force,
            self.left_upper_arm_target_force,
            interp_factor
        )
        self.right_upper_arm_force = self._lerp_tuple(
            self.right_upper_arm_force,
            self.right_upper_arm_target_force,
            interp_factor
        )
        self.left_upper_leg_force = self._lerp_tuple(
            self.left_upper_leg_force,
            self.left_upper_leg_target_force,
            interp_factor
        )
        self.right_upper_leg_force = self._lerp_tuple(
            self.right_upper_leg_force,
            self.right_upper_leg_target_force,
            interp_factor
        )
    
    def update(self, dt):
        """
        Update the physics simulation
        
        Args:
            dt: Delta time in seconds
        """
        # Update gravity to match parameter (allows dynamic adjustment)
        self.space.gravity = (0, self.gravity_strength)
        
        # Generate random forces if enabled
        if self.use_random_forces:
            self._generate_random_forces()
        
        # Smoothly interpolate forces toward targets
        self._smooth_forces(dt)
        
        # Apply forces to upper arms and thighs (where sensors will be)
        # Use .get() with fallback to avoid errors if bodies are missing
        left_arm_body = self.bodies.get('left_upper_arm')
        if left_arm_body:
            left_arm_body.apply_force_at_local_point(
                (self.left_upper_arm_force[0], self.left_upper_arm_force[1]),
                (0, 0)  # Apply at center of body
            )
            left_arm_body.torque += self.left_upper_arm_force[2]
        
        right_arm_body = self.bodies.get('right_upper_arm')
        if right_arm_body:
            right_arm_body.apply_force_at_local_point(
                (self.right_upper_arm_force[0], self.right_upper_arm_force[1]),
                (0, 0)
            )
            right_arm_body.torque += self.right_upper_arm_force[2]
        
        left_leg_body = self.bodies.get('left_thigh')
        if left_leg_body:
            left_leg_body.apply_force_at_local_point(
                (self.left_upper_leg_force[0], self.left_upper_leg_force[1]),
                (0, 0)
            )
            left_leg_body.torque += self.left_upper_leg_force[2]
        
        right_leg_body = self.bodies.get('right_thigh')
        if right_leg_body:
            right_leg_body.apply_force_at_local_point(
                (self.right_upper_leg_force[0], self.right_upper_leg_force[1]),
                (0, 0)
            )
            right_leg_body.torque += self.right_upper_leg_force[2]
        
        # Enforce maximum distances between connected parts (prevents "silly string" effect)
        self._enforce_max_distances()
        
        # Apply normalizing forces to all bodies (helps stabilize and keep things centered)
        if self.apply_normalizing_forces:
            self._apply_normalizing_forces(dt)
        
        # Enforce rotation limits on torso BEFORE physics step (max ±100 degrees)
        self._enforce_torso_rotation_limit()
        
        # Keep head rigidly attached to torso (match angle every frame)
        self._enforce_rigid_head_attachment()
        
        # Maintain rest pose to prevent drooping (apply before damping)
        if self.maintain_rest_pose:
            self._maintain_rest_pose(dt)
        
        # Apply damping to all bodies
        self._apply_damping(dt)
        
        # Step the physics simulation
        self.space.step(dt)
        
        # Enforce rotation limits AGAIN AFTER physics step to catch any overshoot
        self._enforce_torso_rotation_limit()
        
        # Enforce shoulder rotation limits to prevent arms wrapping around body
        self._enforce_shoulder_rotation_limits()
    
    def _vec_to_tuple(self, vec):
        """Convert pymunk Vec2d to tuple of floats"""
        if vec is None:
            return (0.0, 0.0)
        try:
            # Handle pymunk Vec2d
            if hasattr(vec, 'x') and hasattr(vec, 'y'):
                return (float(vec.x), float(vec.y))
            # If it's already a tuple or list
            elif hasattr(vec, '__len__') and len(vec) >= 2:
                return (float(vec[0]), float(vec[1]))
            else:
                return (0.0, 0.0)
        except (AttributeError, TypeError, ValueError) as e:
            # Fallback to default if conversion fails
            print(f"Warning: Failed to convert vec to tuple: {vec}, error: {e}")
            return (0.0, 0.0)
    
    def get_joint_positions(self):
        """
        Get current positions of all joints based on physics bodies
        Returns a dictionary with all joint coordinates
        """
        positions = {}
        
        # Get scaled limb lengths
        scaled_torso = self._apply_scale(self.torso_length)
        scaled_shoulder_width = self._apply_scale(self.shoulder_width / 2)
        
        # Head - safely convert Vec2d to tuple
        head_body = self.bodies.get('head')
        if head_body:
            positions['head_center'] = self._vec_to_tuple(head_body.position)
        else:
            positions['head_center'] = (self.x, self.y)
        
        # Neck (top of torso, where head connects)
        torso_body = self.bodies.get('torso')
        if torso_body:
            neck_top = torso_body.local_to_world((0, -scaled_torso/2))
            positions['neck'] = self._vec_to_tuple(neck_top)
        else:
            positions['neck'] = (self.x, self.y)
        
        # Shoulders (left and right shoulder positions - from triangle base corners)
        if torso_body:
            # Get shoulder positions from triangle base corners (top vertices)
            scaled_shoulder_width = self._apply_scale(self.shoulder_width)
            left_shoulder_local = (-scaled_shoulder_width/2, -scaled_torso/2)  # Left shoulder corner of triangle
            right_shoulder_local = (scaled_shoulder_width/2, -scaled_torso/2)  # Right shoulder corner of triangle
            left_shoulder_pos = torso_body.local_to_world(left_shoulder_local)
            right_shoulder_pos = torso_body.local_to_world(right_shoulder_local)
            positions['left_shoulder'] = self._vec_to_tuple(left_shoulder_pos)
            positions['right_shoulder'] = self._vec_to_tuple(right_shoulder_pos)
            # Shoulder center is midpoint of triangle base
            shoulder_center_local = (0, -scaled_torso/2)
            shoulder_center_pos = torso_body.local_to_world(shoulder_center_local)
            positions['shoulder_center'] = self._vec_to_tuple(shoulder_center_pos)
        else:
            # Fallback if torso body doesn't exist
            scaled_shoulder_width = self._apply_scale(self.shoulder_width)
            positions['left_shoulder'] = (self.x - scaled_shoulder_width/2, self.y - scaled_torso/2)
            positions['right_shoulder'] = (self.x + scaled_shoulder_width/2, self.y - scaled_torso/2)
            positions['shoulder_center'] = (self.x, self.y - scaled_torso/2)
        
        # Waist (bottom of torso)
        if torso_body:
            waist_pos = torso_body.local_to_world((0, scaled_torso/2))
            positions['waist'] = self._vec_to_tuple(waist_pos)
        else:
            positions['waist'] = (self.x, self.y + scaled_torso)
        
        # Elbows (end of upper arms)
        left_arm_body = self.bodies.get('left_upper_arm')
        if left_arm_body:
            left_arm_length = self.limb_lengths.get('left_upper_arm', self._apply_scale(self.upper_arm_length))
            left_elbow_pos = left_arm_body.local_to_world((left_arm_length/2, 0))
            positions['left_elbow'] = self._vec_to_tuple(left_elbow_pos)
        else:
            positions['left_elbow'] = positions.get('left_shoulder', (self.x, self.y))
        
        right_arm_body = self.bodies.get('right_upper_arm')
        if right_arm_body:
            right_arm_length = self.limb_lengths.get('right_upper_arm', self._apply_scale(self.upper_arm_length))
            right_elbow_pos = right_arm_body.local_to_world((right_arm_length/2, 0))
            positions['right_elbow'] = self._vec_to_tuple(right_elbow_pos)
        else:
            positions['right_elbow'] = positions.get('right_shoulder', (self.x, self.y))
        
        # Wrists (end of forearms)
        left_forearm_body = self.bodies.get('left_forearm')
        if left_forearm_body:
            left_forearm_length = self.limb_lengths.get('left_forearm', self._apply_scale(self.forearm_length))
            left_wrist_pos = left_forearm_body.local_to_world((left_forearm_length/2, 0))
            positions['left_wrist'] = self._vec_to_tuple(left_wrist_pos)
        else:
            positions['left_wrist'] = positions.get('left_elbow', (self.x, self.y))
        
        right_forearm_body = self.bodies.get('right_forearm')
        if right_forearm_body:
            right_forearm_length = self.limb_lengths.get('right_forearm', self._apply_scale(self.forearm_length))
            right_wrist_pos = right_forearm_body.local_to_world((right_forearm_length/2, 0))
            positions['right_wrist'] = self._vec_to_tuple(right_wrist_pos)
        else:
            positions['right_wrist'] = positions.get('right_elbow', (self.x, self.y))
        
        # Hips (connection point - triangle apex where both thighs attach)
        if torso_body:
            # Both thighs attach to triangle apex (bottom point)
            hip_apex_local = (0, scaled_torso/2)  # Triangle apex
            hip_apex_pos = torso_body.local_to_world(hip_apex_local)
            # Use same point for both hips (they share the apex)
            positions['left_hip'] = self._vec_to_tuple(hip_apex_pos)
            positions['right_hip'] = self._vec_to_tuple(hip_apex_pos)
        else:
            # Fallback if torso body doesn't exist
            positions['left_hip'] = (self.x, self.y + scaled_torso/2)
            positions['right_hip'] = (self.x, self.y + scaled_torso/2)
        
        # Knees (end of thighs)
        left_thigh_body = self.bodies.get('left_thigh')
        if left_thigh_body:
            left_thigh_length = self.limb_lengths.get('left_thigh', self._apply_scale(self.thigh_length))
            left_knee_pos = left_thigh_body.local_to_world((left_thigh_length/2, 0))
            positions['left_knee'] = self._vec_to_tuple(left_knee_pos)
        else:
            positions['left_knee'] = positions.get('left_hip', (self.x, self.y))
        
        right_thigh_body = self.bodies.get('right_thigh')
        if right_thigh_body:
            right_thigh_length = self.limb_lengths.get('right_thigh', self._apply_scale(self.thigh_length))
            right_knee_pos = right_thigh_body.local_to_world((right_thigh_length/2, 0))
            positions['right_knee'] = self._vec_to_tuple(right_knee_pos)
        else:
            positions['right_knee'] = positions.get('right_hip', (self.x, self.y))
        
        # Ankles (end of shins)
        left_shin_body = self.bodies.get('left_shin')
        if left_shin_body:
            left_shin_length = self.limb_lengths.get('left_shin', self._apply_scale(self.shin_length))
            left_ankle_pos = left_shin_body.local_to_world((left_shin_length/2, 0))
            positions['left_ankle'] = self._vec_to_tuple(left_ankle_pos)
        else:
            positions['left_ankle'] = positions.get('left_knee', (self.x, self.y))
        
        right_shin_body = self.bodies.get('right_shin')
        if right_shin_body:
            right_shin_length = self.limb_lengths.get('right_shin', self._apply_scale(self.shin_length))
            right_ankle_pos = right_shin_body.local_to_world((right_shin_length/2, 0))
            positions['right_ankle'] = self._vec_to_tuple(right_ankle_pos)
        else:
            positions['right_ankle'] = positions.get('right_knee', (self.x, self.y))
        
        return positions
    
    def draw(self, surface):
        """Draw the stick figure on the given surface"""
        try:
            positions = self.get_joint_positions()
        except Exception as e:
            print(f"Error getting joint positions: {e}")
            return  # Skip drawing if positions can't be calculated
        
        # Draw head (circle) - with safety checks
        try:
            head_center = positions.get('head_center', (self.x, self.y))
            # Ensure head_center is a valid tuple of numbers
            if not isinstance(head_center, (tuple, list)) or len(head_center) < 2:
                head_center = (float(head_center[0]) if hasattr(head_center, '__getitem__') else self.x, 
                              float(head_center[1]) if hasattr(head_center, '__getitem__') else self.y)
            else:
                head_center = (float(head_center[0]), float(head_center[1]))
            
            # Draw head with visual radius (not collision radius, which is larger)
            head_radius = self._apply_scale(self.head_radius)
            pygame.draw.circle(surface, self.color,
                              (int(head_center[0]), int(head_center[1])),
                              int(head_radius), self.line_width)
            # Optional: Draw collision buffer zone in a different color for debugging
            # collision_radius = head_radius + self._apply_scale(self.collision_buffer_zone)
            # pygame.draw.circle(surface, (100, 100, 100, 50),  # Semi-transparent gray
            #                   (int(head_center[0]), int(head_center[1])),
            #                   int(collision_radius), 1)
        except Exception as e:
            print(f"Error drawing head: {e}, head_center: {head_center if 'head_center' in locals() else 'unknown'}")
        
        # Calculate visual thickness for drawing - match collision thickness for visual accuracy
        # Collision thickness = limb_thickness (12.0) + collision_buffer_zone (6.0) = 18.0 total
        # Visual should match collision for better representation
        collision_thickness_total = self.limb_thickness + self.collision_buffer_zone  # Total collision radius
        visual_thickness = max(self.line_width, int(self._apply_scale(collision_thickness_total * 0.6)))  # 60% of collision for visual
        
        # Draw torso as a triangle (T-shaped body)
        # Calculate triangle vertices in world coordinates
        torso_body = self.bodies.get('torso')
        if torso_body and 'neck' in positions and 'waist' in positions:
            try:
                # Get triangle vertices from torso body shape
                # Triangle: wide base at top (shoulders), narrows to point at bottom (hips)
                scaled_torso = self._apply_scale(self.torso_length)
                scaled_shoulder_width = self._apply_scale(self.shoulder_width)
                
                # Local triangle vertices (same as in physics shape - inverted isosceles)
                local_vertices = [
                    (-scaled_shoulder_width/2, -scaled_torso/2),  # Left shoulder (top left corner)
                    (scaled_shoulder_width/2, -scaled_torso/2),  # Right shoulder (top right corner)
                    (0, scaled_torso/2),  # Hips (bottom point/apex)
                ]
                
                # Transform to world coordinates
                world_vertices = []
                for local_vertex in local_vertices:
                    world_vertex = torso_body.local_to_world(local_vertex)
                    world_vertices.append((int(world_vertex.x), int(world_vertex.y)))
                
                # Draw filled triangle
                pygame.draw.polygon(surface, self.color, world_vertices)
                # Draw triangle outline
                pygame.draw.polygon(surface, self.color, world_vertices, visual_thickness // 2)
            except (AttributeError, TypeError, ValueError) as e:
                # Fallback to line drawing if triangle calculation fails
                pygame.draw.line(surface, self.color,
                                positions['neck'],
                                positions['waist'],
                                visual_thickness)
        else:
            # Fallback to line drawing if body not available
            pygame.draw.line(surface, self.color,
                            positions.get('neck', (self.x, self.y)),
                            positions.get('waist', (self.x, self.y + self._apply_scale(self.torso_length))),
                            visual_thickness)
        
        # No separate shoulder line - triangle base IS the shoulders
        # Draw left arm (upper arm + forearm) with thickness
        pygame.draw.line(surface, self.color,
                        positions['left_shoulder'],
                        positions['left_elbow'],
                        visual_thickness)
        pygame.draw.line(surface, self.color,
                        positions['left_elbow'],
                        positions['left_wrist'],
                        visual_thickness)
        
        # Draw right arm (upper arm + forearm) with thickness
        pygame.draw.line(surface, self.color,
                        positions['right_shoulder'],
                        positions['right_elbow'],
                        visual_thickness)
        pygame.draw.line(surface, self.color,
                        positions['right_elbow'],
                        positions['right_wrist'],
                        visual_thickness)
        
        # Draw left leg (thigh + shin) with thickness
        pygame.draw.line(surface, self.color,
                        positions['left_hip'],
                        positions['left_knee'],
                        visual_thickness)
        pygame.draw.line(surface, self.color,
                        positions['left_knee'],
                        positions['left_ankle'],
                        visual_thickness)
        
        # Draw right leg (thigh + shin) with thickness
        pygame.draw.line(surface, self.color,
                        positions['right_hip'],
                        positions['right_knee'],
                        visual_thickness)
        pygame.draw.line(surface, self.color,
                        positions['right_knee'],
                        positions['right_ankle'],
                        visual_thickness)
        
        # Draw acceleration visualizations if enabled
        if self.show_accelerations:
            self._draw_accelerations(surface, positions)
        
        # Draw torso rotation visualization if enabled
        if self.show_torso_rotation:
            self._draw_torso_rotation(surface)
    
    def _draw_accelerations(self, surface, positions):
        """Draw acceleration visualization: circles at force points and arrows for 3D forces"""
        # Calculate midpoints of upper arms and upper legs (where forces are applied)
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
        
        # Draw circles and arrows for each force point
        # Convert forces to acceleration-like visualization
        self._draw_force_point(surface, left_upper_arm_mid, self.left_upper_arm_force)
        self._draw_force_point(surface, right_upper_arm_mid, self.right_upper_arm_force)
        self._draw_force_point(surface, left_upper_leg_mid, self.left_upper_leg_force)
        self._draw_force_point(surface, right_upper_leg_mid, self.right_upper_leg_force)
    
    def _draw_force_point(self, surface, position, force):
        """Draw a circle at the force point and arrows for 3D force"""
        x, y = int(position[0]), int(position[1])
        fx, fy, torque = force
        
        # Draw circle at force point
        circle_radius = int(self._apply_scale(self.accel_circle_radius))
        pygame.draw.circle(surface, self.accel_circle_color, (x, y), circle_radius, 2)
        
        # Calculate force magnitude
        force_magnitude = math.sqrt(fx**2 + fy**2 + abs(torque))
        if force_magnitude < 1.0:  # Don't draw if force is too small
            return
        
        # Scale arrow length
        base_length = self._apply_scale(20)
        arrow_length = base_length * force_magnitude / (self.random_force_strength * 0.5) * self.accel_arrow_length_scale
        arrow_length = max(5, min(arrow_length, base_length * 3))
        
        # Draw X component arrow (horizontal, red)
        if abs(fx) > 1.0:
            x_arrow_length = arrow_length * abs(fx) / force_magnitude
            x_direction = 1 if fx > 0 else -1
            end_x = x + x_arrow_length * x_direction
            self._draw_arrow(surface, (x, y), (end_x, y), (255, 100, 100), 2)
        
        # Draw Y component arrow (vertical, green)
        if abs(fy) > 1.0:
            y_arrow_length = arrow_length * abs(fy) / force_magnitude
            y_direction = 1 if fy > 0 else -1
            end_y = y + y_arrow_length * y_direction
            self._draw_arrow(surface, (x, y), (x, end_y), (100, 255, 100), 2)
        
        # Draw torque arrow (diagonal/perpendicular, blue)
        if abs(torque) > 1.0:
            torque_arrow_length = arrow_length * abs(torque) / force_magnitude
            torque_direction = 1 if torque > 0 else -1
            end_x = x + torque_arrow_length * torque_direction * math.cos(math.pi / 4)
            end_y = y + torque_arrow_length * torque_direction * math.sin(math.pi / 4)
            self._draw_arrow(surface, (x, y), (end_x, end_y), (100, 100, 255), 2)
    
    def _draw_arrow(self, surface, start, end, color, width):
        """Draw an arrow from start to end"""
        pygame.draw.line(surface, color, start, end, width)
        
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle = math.atan2(dy, dx)
        
        arrowhead_length = 8
        arrowhead_angle = math.pi / 6
        
        arrowhead_x1 = end[0] - arrowhead_length * math.cos(angle - arrowhead_angle)
        arrowhead_y1 = end[1] - arrowhead_length * math.sin(angle - arrowhead_angle)
        arrowhead_x2 = end[0] - arrowhead_length * math.cos(angle + arrowhead_angle)
        arrowhead_y2 = end[1] - arrowhead_length * math.sin(angle + arrowhead_angle)
        
        pygame.draw.polygon(surface, color, [
            end,
            (arrowhead_x1, arrowhead_y1),
            (arrowhead_x2, arrowhead_y2)
        ])
    
    def _draw_torso_rotation(self, surface):
        """Draw torso rotation visualization: arc showing current angle and rotation limits"""
        torso_body = self.bodies.get('torso')
        if not torso_body or not hasattr(self, 'torso_initial_angle') or not hasattr(self, 'torso_max_rotation'):
            return
        
        try:
            # Get torso center position
            torso_pos = torso_body.position
            center_x = float(torso_pos.x) if hasattr(torso_pos, 'x') else float(torso_pos[0])
            center_y = float(torso_pos.y) if hasattr(torso_pos, 'y') else float(torso_pos[1])
            center = (int(center_x), int(center_y))
            
            # Get current angle and calculate angle difference from initial
            current_angle = torso_body.angle
            angle_diff = current_angle - self.torso_initial_angle
            
            # Normalize angle difference to [-π, π] range
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Calculate rotation limits in radians
            max_rotation_rad = self.torso_max_rotation
            
            # Draw arc radius (scaled)
            arc_radius = int(self._apply_scale(self.rotation_arc_radius))
            
            # Draw rotation limits as semi-transparent arcs (blue)
            # Maximum clockwise limit
            max_cw_angle = self.torso_initial_angle + max_rotation_rad
            max_ccw_angle = self.torso_initial_angle - max_rotation_rad
            
            # Draw limit arcs (only visible portion)
            limit_arc_thickness = 3
            # Clockwise limit
            cw_start_angle_deg = math.degrees(max_cw_angle) - 5
            cw_end_angle_deg = math.degrees(max_cw_angle) + 5
            pygame.draw.arc(surface, self.rotation_limit_color, 
                          (center_x - arc_radius, center_y - arc_radius, 
                           arc_radius * 2, arc_radius * 2),
                          math.radians(cw_start_angle_deg), 
                          math.radians(cw_end_angle_deg),
                          limit_arc_thickness)
            
            # Counter-clockwise limit
            ccw_start_angle_deg = math.degrees(max_ccw_angle) - 5
            ccw_end_angle_deg = math.degrees(max_ccw_angle) + 5
            pygame.draw.arc(surface, self.rotation_limit_color, 
                          (center_x - arc_radius, center_y - arc_radius, 
                           arc_radius * 2, arc_radius * 2),
                          math.radians(ccw_start_angle_deg), 
                          math.radians(ccw_end_angle_deg),
                          limit_arc_thickness)
            
            # Draw current rotation arc (red) - arc from initial angle to current angle
            initial_angle_deg = math.degrees(self.torso_initial_angle)
            current_angle_deg = math.degrees(current_angle)
            
            # Draw arc showing rotation from initial to current
            # Pygame's arc draws from start_angle to end_angle, counter-clockwise
            # We want to show the difference, so draw from initial to current
            if abs(angle_diff) > 0.01:  # Only draw if there's significant rotation
                # Determine start and end angles for the arc
                if angle_diff > 0:
                    # Clockwise rotation (angle_diff is positive)
                    arc_start = math.radians(initial_angle_deg)
                    arc_end = math.radians(current_angle_deg)
                else:
                    # Counter-clockwise rotation (angle_diff is negative)
                    arc_start = math.radians(current_angle_deg)
                    arc_end = math.radians(initial_angle_deg)
                
                pygame.draw.arc(surface, self.rotation_arc_color,
                              (center_x - arc_radius, center_y - arc_radius,
                               arc_radius * 2, arc_radius * 2),
                              arc_start, arc_end,
                              4)  # Thicker line for current rotation
            
            # Draw arrow from center showing current rotation direction and magnitude
            if abs(angle_diff) > 0.01:
                # Calculate arrow endpoint based on current angle
                arrow_length = arc_radius * 0.7
                # Initial angle is π/2 (vertical up), so subtract π/2 to align with vertical
                arrow_end_x = center_x + arrow_length * math.cos(current_angle - math.pi / 2)
                arrow_end_y = center_y + arrow_length * math.sin(current_angle - math.pi / 2)
                arrow_end = (int(arrow_end_x), int(arrow_end_y))
                
                # Draw arrow
                self._draw_arrow(surface, center, arrow_end, self.rotation_arc_color, 3)
            
            # Draw text showing angle in degrees - make it more visible
            angle_deg = math.degrees(angle_diff)
            
            # Create font for text display
            try:
                # Try to use a system font first, fallback to default
                font = pygame.font.Font(None, 32)  # Larger font for better visibility
            except:
                try:
                    font = pygame.font.SysFont('Arial', 28)
                except:
                    font = pygame.font.Font(pygame.font.get_default_font(), 24)
            
            if font:
                # Draw rotation value text with background for better visibility
                angle_text = f"Rotation: {angle_deg:+.1f}°"  # + sign shows direction
                
                # Render text with anti-aliasing
                text_surface = font.render(angle_text, True, self.rotation_arc_color)
                text_x = int(center_x + arc_radius + 15)
                text_y = int(center_y - 40)
                
                # Draw background rectangle for text readability (solid black, no alpha)
                bg_rect = pygame.Rect(text_x - 5, text_y - 2, text_surface.get_width() + 10, text_surface.get_height() + 4)
                pygame.draw.rect(surface, (0, 0, 0), bg_rect)  # Black background
                
                surface.blit(text_surface, (text_x, text_y))
                
                # Draw text showing max limits
                max_limit_text = f"Max: ±{math.degrees(max_rotation_rad):.0f}°"
                limit_text_surface = font.render(max_limit_text, True, self.rotation_limit_color)
                limit_bg_rect = pygame.Rect(text_x - 5, text_y + 28, limit_text_surface.get_width() + 10, limit_text_surface.get_height() + 4)
                pygame.draw.rect(surface, (0, 0, 0), limit_bg_rect)  # Black background
                surface.blit(limit_text_surface, (text_x, text_y + 30))
                
                # Also draw absolute current angle from vertical (0° = vertical)
                current_abs_angle = math.degrees(current_angle) % 360
                abs_angle_text = f"Angle: {current_abs_angle:.1f}°"
                abs_text_surface = font.render(abs_angle_text, True, (200, 200, 200))  # Gray color
                abs_bg_rect = pygame.Rect(text_x - 5, text_y + 58, abs_text_surface.get_width() + 10, abs_text_surface.get_height() + 4)
                pygame.draw.rect(surface, (0, 0, 0), abs_bg_rect)  # Black background
                surface.blit(abs_text_surface, (text_x, text_y + 60))
            
        except (AttributeError, TypeError, ValueError, OverflowError) as e:
            # Skip if calculation fails
            pass
    
    def cleanup(self):
        """Clean up physics resources"""
        if self.owns_space and self.space:
            # Remove all constraints (springs, etc.)
            for constraint in list(self.constraints.values()):
                try:
                    self.space.remove(constraint)
                except:
                    pass
            
            # Remove all joints
            for joint in list(self.joints.values()):
                try:
                    self.space.remove(joint)
                except:
                    pass
            
            # Remove all bodies and shapes from space (except static pivot)
            for name, body in list(self.bodies.items()):
                if name == 'center_pivot':
                    continue  # Skip static body
                try:
                    for shape in body.shapes:
                        self.space.remove(shape)
                    self.space.remove(body)
                except:
                    pass
            
            # Space will be garbage collected
            self.space = None
