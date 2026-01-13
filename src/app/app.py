"""Top-level application orchestration."""
from __future__ import annotations

from dataclasses import dataclass

import pygame

from core.config import AppConfig, ArduinoConfig
from input.touch import TouchInput
from input.sensors import SensorHub
from physics.particles import ParticleSystem
from rendering.renderer import Renderer
from utils.clock import FrameClock


@dataclass
class TheFoxApp:
    app_config: AppConfig
    arduino_config: ArduinoConfig

    def __post_init__(self) -> None:
        pygame.init()
        pygame.display.set_caption(self.app_config.window.title)

        flags = pygame.FULLSCREEN if self.app_config.window.fullscreen else 0
        self.screen = pygame.display.set_mode(self.app_config.window.size, flags)

        self.clock = FrameClock(target_fps=self.app_config.window.target_fps)
        self.touch = TouchInput()
        self.sensors = SensorHub(self.arduino_config)
        self.particles = ParticleSystem(config=self.app_config.particles)
        self.renderer = Renderer(config=self.app_config, screen=self.screen)

    def run(self) -> None:
        running = True
        while running:
            dt = self.clock.tick()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                self.touch.handle_event(event)

            sensor_state = self.sensors.poll()
            self.particles.update(dt, self.touch.state, sensor_state)

            self.renderer.draw(
                particles=self.particles,
                touch_state=self.touch.state,
                sensor_state=sensor_state,
            )

            pygame.display.flip()

        self.sensors.shutdown()
        pygame.quit()
