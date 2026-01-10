\
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple, List

import pymunk

Vec2 = Tuple[float, float]

SENSORS = ("left_upper_arm", "right_upper_arm", "left_thigh", "right_thigh")


@dataclass
class StickFigureConfig:
    max_torso_tilt_deg: float = 45.0
    anti_flip_deg: float = 85.0
    limb_cross_threshold: float = 4.0
    limb_corrective_force: float = 1500.0
    limb_soft_force: float = 400.0

    yaw_gain: float = 0.0022   # converts "spin couple" into yaw change
    yaw_damping: float = 0.96  # per second-ish damping applied in update


class StickFigure:
    """
    Physics rig:
      - a torso body (box)
      - four sensor point bodies (small circles)
      - each sensor is connected to torso with a damped spring + pivot
    Constraints:
      - torso tilt clamped via rotary limit relative to a static anchor
      - anti-flip enforced via additional angular damping/restore
    Yaw:
      - simulated "front/back spin" tracked as self.yaw_deg, driven only by sensor forces pattern.
        (In pure 2D, yaw isn't a separate axis of rotation; this is a UI/logic layer.)
    """
    def __init__(self, space: pymunk.Space, pos=(400, 300), cfg: StickFigureConfig | None = None):
        self.space = space
        self.cfg = cfg or StickFigureConfig()

        self.joints: List[pymunk.Constraint] = []
        self.torso = self._make_torso(pos)
        self.anchor = self._make_anchor(pos)

        self.sensors: Dict[str, pymunk.Body] = self._make_sensors(pos)
        self._connect_rig()

        # Separate yaw indicator (0..360)
        self.yaw_deg = 0.0
        self._yaw_vel = 0.0

    def _make_torso(self, pos) -> pymunk.Body:
        mass = 10.0
        size = (28, 72)
        moment = pymunk.moment_for_box(mass, size)
        body = pymunk.Body(mass, moment)
        body.position = pos
        shape = pymunk.Poly.create_box(body, size)
        shape.friction = 0.8
        shape.elasticity = 0.0
        self.space.add(body, shape)
        return body

    def _make_anchor(self, pos) -> pymunk.Body:
        # Static reference body for angle-limiting the torso
        anchor = self.space.static_body
        # Use a pivot joint to keep torso in place-ish (still can move from forces)
        pj = pymunk.PivotJoint(self.torso, anchor, pos)
        pj.max_force = 50000
        pj.error_bias = pow(1.0 - 0.25, 60)  # soft-ish
        self.space.add(pj)
        self.joints.append(pj)

        # Rotary limit for tilt
        max_tilt = math.radians(self.cfg.max_torso_tilt_deg)
        rlim = pymunk.RotaryLimitJoint(self.torso, anchor, -max_tilt, max_tilt)
        self.space.add(rlim)
        self.joints.append(rlim)

        return anchor

    def _make_sensors(self, pos) -> Dict[str, pymunk.Body]:
        offsets = {
            "left_upper_arm": (-28, 18),
            "right_upper_arm": (28, 18),
            "left_thigh": (-16, -34),
            "right_thigh": (16, -34),
        }
        out: Dict[str, pymunk.Body] = {}
        for name, (ox, oy) in offsets.items():
            mass = 1.0
            moment = pymunk.moment_for_circle(mass, 0, 6)
            b = pymunk.Body(mass, moment)
            b.position = (pos[0] + ox, pos[1] + oy)
            c = pymunk.Circle(b, 6)
            c.friction = 0.7
            c.elasticity = 0.0
            # Give sensors their own collision group to reduce self-jitter
            c.filter = pymunk.ShapeFilter(group=1)
            self.space.add(b, c)
            out[name] = b
        return out

    def _connect_rig(self) -> None:
        # Connect sensors to torso with springs and pivots
        for name, sensor in self.sensors.items():
            # Local anchors on torso
            if "upper_arm" in name:
                local = (-20, 18) if "left" in name else (20, 18)
            else:
                local = (-12, -32) if "left" in name else (12, -32)

            pivot = pymunk.PivotJoint(sensor, self.torso, sensor.position)
            pivot.max_force = 35000
            self.space.add(pivot)
            self.joints.append(pivot)

            spring = pymunk.DampedSpring(
                sensor, self.torso,
                (0, 0), local,
                rest_length=0.0,
                stiffness=1800,
                damping=140,
            )
            self.space.add(spring)
            self.joints.append(spring)

    def sensor_bodies(self) -> Dict[str, pymunk.Body]:
        return dict(self.sensors)

    def reset_pose(self, pos=(400, 300)) -> None:
        self.torso.position = pos
        self.torso.velocity = (0, 0)
        self.torso.angle = 0.0
        self.torso.angular_velocity = 0.0

        offsets = {
            "left_upper_arm": (-28, 18),
            "right_upper_arm": (28, 18),
            "left_thigh": (-16, -34),
            "right_thigh": (16, -34),
        }
        for name, (ox, oy) in offsets.items():
            b = self.sensors[name]
            b.position = (pos[0] + ox, pos[1] + oy)
            b.velocity = (0, 0)
            b.angle = 0.0
            b.angular_velocity = 0.0

        self.yaw_deg = 0.0
        self._yaw_vel = 0.0

    def enforce_anti_flip(self) -> None:
        # RotaryLimitJoint already clamps tilt; this adds extra resistance near the edges.
        max_safe = math.radians(self.cfg.anti_flip_deg)
        a = self.torso.angle
        if abs(a) > max_safe:
            # Strongly damp angular velocity when near "ass up"
            self.torso.angular_velocity *= 0.2
            # Pull back toward 0
            self.torso.torque += -a * 35000

    def enforce_limb_sides(self) -> None:
        torso_x = self.torso.position.x
        th = self.cfg.limb_cross_threshold

        for name, b in self.sensors.items():
            dx = b.position.x - torso_x
            # Left sensors must remain <= torso_x - th
            if "left" in name and dx > -th:
                # corrective push left
                mag = self.cfg.limb_corrective_force if dx > 0 else self.cfg.limb_soft_force
                b.apply_force_at_world_point((-mag, 0), b.position)
            # Right sensors must remain >= torso_x + th
            if "right" in name and dx < th:
                mag = self.cfg.limb_corrective_force if dx < 0 else self.cfg.limb_soft_force
                b.apply_force_at_world_point((mag, 0), b.position)

    def update_yaw_from_sensor_forces(self, applied_forces: List[tuple]) -> None:
        """
        Compute a 'spin couple' signal from the applied sensor forces.
        This drives yaw ONLY through forces on sensors (never torso torque).
        """
        # Map body -> sensor name for quick lookup
        inv = {b: name for name, b in self.sensors.items()}

        left_fx = 0.0
        right_fx = 0.0
        for body, (fx, fy) in applied_forces:
            name = inv.get(body)
            if not name:
                continue
            if "left" in name:
                left_fx += fx
            else:
                right_fx += fx

        # Spin couple: left pushing right AND right pushing left creates rotation intent.
        # If left_fx is positive and right_fx is negative => clockwise (arbitrary)
        couple = (left_fx - right_fx)

        # Integrate yaw velocity and yaw angle
        self._yaw_vel += couple * self.cfg.yaw_gain
        # damping proportional to dt-ish; apply as exponential approx in caller
        self.yaw_deg = (self.yaw_deg + self._yaw_vel) % 360.0

    def yaw_indicator(self) -> str:
        # Front if yaw within [-90, +90] of 0, else back
        a = self.yaw_deg % 360.0
        return "F" if (a <= 90.0 or a >= 270.0) else "B"
