"""Particle system placeholder for SPH simulation."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from core.config import ParticleConfig
from input.sensors import SensorState
from input.touch import TouchState


@dataclass
class Particle:
    position: List[float]
    velocity: List[float]


@dataclass
class ParticleSystem:
    config: ParticleConfig
    particles: List[Particle] = field(default_factory=list)

    def update(self, dt: float, touch_state: TouchState, sensor_state: SensorState) -> None:
        self._spawn_particles(sensor_state)
        self._apply_touch_forces(touch_state)
        self._integrate(dt)

    def _spawn_particles(self, sensor_state: SensorState) -> None:
        emit_strength = sum(sensor_state.values) * self.config.emitter_strength
        spawn_count = int(min(emit_strength, self.config.max_particles - len(self.particles)))
        for _ in range(spawn_count):
            self.particles.append(Particle(position=[320.0, 240.0], velocity=[0.0, 0.0]))

    def _apply_touch_forces(self, touch_state: TouchState) -> None:
        if not touch_state.positions:
            return
        for particle in self.particles:
            for touch_x, touch_y in touch_state.positions:
                dx = touch_x - particle.position[0]
                dy = touch_y - particle.position[1]
                particle.velocity[0] += dx * 0.001
                particle.velocity[1] += dy * 0.001

    def _integrate(self, dt: float) -> None:
        gravity = self.config.gravity
        for particle in self.particles:
            particle.velocity[1] += gravity * dt
            particle.position[0] += particle.velocity[0] * dt
            particle.position[1] += particle.velocity[1] * dt
