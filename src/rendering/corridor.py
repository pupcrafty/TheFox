"""Pseudo-3D corridor renderer."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

import pygame

from core.config import CorridorConfig
from rendering.tiles import TileAtlas


@dataclass
class CorridorRenderer:
    config: CorridorConfig
    screen: pygame.Surface

    def __post_init__(self) -> None:
        self._offset = 0.0
        self._tile_atlas = TileAtlas.from_assets()
        self._tile_scroll = 0

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

        self._draw_tile_preview(width, height)

    def _draw_tile_preview(self, width: int, height: int) -> None:
        if not self._tile_atlas.tiles:
            return
        margin = 24
        scale = 0.25
        columns = 4
        rows = 5
        self._tile_scroll = (self._tile_scroll + 1) % len(self._tile_atlas.tiles)
        tile_height = int(self._tile_atlas.tiles[0].metadata.sprite_height * scale)
        padding = 4
        self._tile_atlas.draw_grid(
            target=self.screen,
            origin=(margin, height - margin - rows * tile_height - (rows - 1) * padding),
            columns=columns,
            rows=rows,
            scale=scale,
            start_index=self._tile_scroll,
            padding=padding,
        )

    def _create_decagon(self, center: Tuple[int, int], radius: float) -> List[Tuple[int, int]]:
        cx, cy = center
        points = []
        for i in range(10):
            angle = (i / 10.0) * 6.283185307179586
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((int(x), int(y)))
        return points
