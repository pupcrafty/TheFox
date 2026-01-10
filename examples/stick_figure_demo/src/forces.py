\
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

Vec2 = Tuple[float, float]

VALID_TARGETS = ("left_upper_arm", "right_upper_arm", "left_thigh", "right_thigh")


@dataclass(frozen=True)
class ForceDef:
    time: float
    target: str
    force: Vec2
    duration: float
    fade_type: str = "linear"


@dataclass
class ActiveForce:
    body: "pymunk.Body"
    force: Vec2
    duration: float
    fade_type: str
    elapsed: float = 0.0

    def strength(self) -> float:
        # Force applies for a moment then fades out naturally.
        if self.duration <= 1e-9:
            return 0.0
        t = self.elapsed / self.duration
        if t >= 1.0:
            return 0.0

        if self.fade_type == "instant":
            return 1.0 if self.elapsed < self.duration else 0.0
        if self.fade_type == "exponential":
            # Strong initial impulse; quickly decays.
            return math.exp(-4.0 * t)
        # default linear
        return max(0.0, 1.0 - t)


class ForceManager:
    """
    Applies forces to pymunk bodies. This class does NOT decide which bodies are valid;
    callers must enforce targeting rules.
    """
    def __init__(self):
        self.active: List[ActiveForce] = []
        self._last_applied_for_debug: List[Tuple[object, Vec2]] = []

    def apply(self, af: ActiveForce) -> None:
        self.active.append(af)

    def update(self, dt: float) -> None:
        self._last_applied_for_debug.clear()

        for af in self.active[:]:
            af.elapsed += dt
            s = af.strength()
            if s <= 0.0:
                self.active.remove(af)
                continue

            fx, fy = af.force
            f = (fx * s, fy * s)
            # Apply at center of mass (sensor point body)
            af.body.apply_force_at_local_point(f, (0, 0))
            self._last_applied_for_debug.append((af.body, f))

    def last_applied(self) -> List[Tuple[object, Vec2]]:
        """Used by StickFigure to infer 'spin' input for yaw simulation."""
        return list(self._last_applied_for_debug)


class ForceSequenceLibrary:
    def __init__(self, sequences: Dict[str, List[ForceDef]]):
        self.sequences = sequences

    @staticmethod
    def load(path: str) -> "ForceSequenceLibrary":
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)

        sequences: Dict[str, List[ForceDef]] = {}
        for seq in raw.get("sequences", []):
            name = seq["name"]
            forces: List[ForceDef] = []
            for item in seq.get("forces", []):
                target = item["target"]
                if target not in VALID_TARGETS:
                    raise ValueError(f"Invalid target in JSON: {target}. Must be one of {VALID_TARGETS}")
                forces.append(
                    ForceDef(
                        time=float(item["time"]),
                        target=target,
                        force=(float(item["force"][0]), float(item["force"][1])),
                        duration=float(item["duration"]),
                        fade_type=str(item.get("fade_type", "linear")),
                    )
                )
            forces.sort(key=lambda x: x.time)
            sequences[name] = forces

        return ForceSequenceLibrary(sequences)


class SequencePlayer:
    def __init__(self, library: ForceSequenceLibrary, manager: ForceManager):
        self.library = library
        self.manager = manager
        self.current: Optional[str] = None
        self.t = 0.0
        self._cursor = 0

    def play(self, name: str) -> None:
        if name not in self.library.sequences:
            raise KeyError(f"Unknown sequence: {name}")
        self.current = name
        self.t = 0.0
        self._cursor = 0

    def stop(self) -> None:
        self.current = None
        self.t = 0.0
        self._cursor = 0

    def update(self, dt: float, target_to_body: Dict[str, "pymunk.Body"]) -> None:
        if not self.current:
            return

        self.t += dt
        seq = self.library.sequences[self.current]

        # Fire any ForceDefs whose time has arrived.
        while self._cursor < len(seq) and seq[self._cursor].time <= self.t:
            fd = seq[self._cursor]
            self._cursor += 1

            body = target_to_body[fd.target]  # targets validated at load time
            self.manager.apply(
                ActiveForce(body=body, force=fd.force, duration=fd.duration, fade_type=fd.fade_type)
            )

        # Auto-stop when done and no forces remain to be scheduled.
        if self._cursor >= len(seq):
            # Let active forces fade out naturally; stop scheduling new ones.
            self.current = None
