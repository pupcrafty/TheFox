"""Pseudo-3D corridor renderer."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

import pygame

from core.config import CorridorConfig


@dataclass
class CorridorRenderer:
    config: CorridorConfig
    screen: pygame.Surface

    def __post_init__(self) -> None:
        self._offset = 0.0

    def draw(self) -> None:
        width, height = self.screen.get_size()
        center = (width // 2, height // 2)
        color = self.config.wall_color

        self._offset = (self._offset + self.config.scroll_speed) % self.config.segment_spacing
        for index in range(self.config.segment_count):
            depth = (index * self.config.segment_spacing + self._offset) / self.config.segment_count
            scale = 1.0 - depth * 0.8
            polygon = self._create_decagon(center, min(width, height) * 0.9 * scale)
            pygame.draw.polygon(self.screen, color, polygon, width=2)

    def _create_decagon(self, center: Tuple[int, int], radius: float) -> List[Tuple[int, int]]:
        cx, cy = center
        points = []
        for i in range(10):
            angle = (i / 10.0) * 6.283185307179586
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((int(x), int(y)))
        return points
