"""Frame timing utilities."""
from __future__ import annotations

from dataclasses import dataclass

import pygame


@dataclass
class FrameClock:
    target_fps: int

    def __post_init__(self) -> None:
        self._clock = pygame.time.Clock()

    def tick(self) -> float:
        return self._clock.tick(self.target_fps) / 1000.0
