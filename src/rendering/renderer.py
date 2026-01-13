"""Layered renderer for the corridor and particle system."""
from __future__ import annotations

from dataclasses import dataclass
import pygame

from core.config import AppConfig
from input.sensors import SensorState
from input.touch import TouchState
from physics.particles import ParticleSystem
from rendering.corridor import CorridorRenderer


@dataclass
class Renderer:
    config: AppConfig
    screen: pygame.Surface

    def __post_init__(self) -> None:
        self.corridor = CorridorRenderer(config=self.config.corridor, screen=self.screen)

    def draw(
        self,
        particles: ParticleSystem,
        touch_state: TouchState,
        sensor_state: SensorState,
    ) -> None:
        self.screen.fill(self.config.window.background_color)
        self.corridor.draw()
        self._draw_particles(particles)
        self._draw_touch_points(touch_state)
        self._draw_sensor_overlay(sensor_state)

    def _draw_particles(self, particles: ParticleSystem) -> None:
        for particle in particles.particles:
            pygame.draw.circle(self.screen, (120, 210, 255), particle.position, 6)

    def _draw_touch_points(self, touch_state: TouchState) -> None:
        for position in touch_state.positions:
            pygame.draw.circle(self.screen, (255, 200, 80), position, 12, width=2)

    def _draw_sensor_overlay(self, sensor_state: SensorState) -> None:
        bar_width = 20
        base_x = 20
        base_y = self.screen.get_height() - 20
        for index, value in enumerate(sensor_state.values):
            height = min(max(value * 50, 0), 150)
            rect = pygame.Rect(base_x + index * (bar_width + 10), base_y - height, bar_width, height)
            pygame.draw.rect(self.screen, (120, 255, 150), rect)
