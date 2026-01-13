"""Configuration models and loaders."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple


@dataclass(frozen=True)
class WindowConfig:
    size: Tuple[int, int]
    fullscreen: bool
    title: str
    target_fps: int
    background_color: Tuple[int, int, int]


@dataclass(frozen=True)
class ParticleConfig:
    max_particles: int
    emitter_strength: float
    gravity: float
    viscosity: float
    cohesion: float


@dataclass(frozen=True)
class CorridorConfig:
    segment_count: int
    segment_spacing: float
    scroll_speed: float
    wall_color: Tuple[int, int, int]


@dataclass(frozen=True)
class AppConfig:
    window: WindowConfig
    particles: ParticleConfig
    corridor: CorridorConfig


@dataclass(frozen=True)
class ArduinoConfig:
    enabled: bool
    port: str
    baud_rate: int
    sensor_count: int


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def load_app_config(path: Path) -> AppConfig:
    payload = _load_json(path)

    window = WindowConfig(
        size=tuple(payload["window"]["size"]),
        fullscreen=payload["window"]["fullscreen"],
        title=payload["window"]["title"],
        target_fps=payload["window"]["target_fps"],
        background_color=tuple(payload["window"]["background_color"]),
    )

    particles = ParticleConfig(
        max_particles=payload["particles"]["max_particles"],
        emitter_strength=payload["particles"]["emitter_strength"],
        gravity=payload["particles"]["gravity"],
        viscosity=payload["particles"]["viscosity"],
        cohesion=payload["particles"]["cohesion"],
    )

    corridor = CorridorConfig(
        segment_count=payload["corridor"]["segment_count"],
        segment_spacing=payload["corridor"]["segment_spacing"],
        scroll_speed=payload["corridor"]["scroll_speed"],
        wall_color=tuple(payload["corridor"]["wall_color"]),
    )

    return AppConfig(window=window, particles=particles, corridor=corridor)


def load_arduino_config(path: Path) -> ArduinoConfig:
    payload = _load_json(path)
    return ArduinoConfig(
        enabled=payload["enabled"],
        port=payload["port"],
        baud_rate=payload["baud_rate"],
        sensor_count=payload["sensor_count"],
    )
