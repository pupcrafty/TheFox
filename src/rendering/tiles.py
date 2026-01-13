"""Sprite tile loading helpers for corridor rendering."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import pygame

from utils.assets import get_asset_path


@dataclass(frozen=True)
class TileMetadata:
    filename: str
    up: Optional[int]
    left: Optional[int]
    down: Optional[int]
    right: Optional[int]
    sprite_x: int
    sprite_y: int
    sprite_width: int
    sprite_height: int

    @property
    def rect(self) -> pygame.Rect:
        return pygame.Rect(
            self.sprite_x,
            self.sprite_y,
            self.sprite_width,
            self.sprite_height,
        )


@dataclass(frozen=True)
class TileSprite:
    metadata: TileMetadata
    surface: pygame.Surface


class TileAtlas:
    def __init__(
        self,
        sprite_sheet: pygame.Surface,
        tiles: Iterable[TileMetadata],
    ) -> None:
        self._sprite_sheet = sprite_sheet
        self._tiles = [TileSprite(metadata=tile, surface=self._extract_tile(tile)) for tile in tiles]

    @property
    def tiles(self) -> List[TileSprite]:
        return self._tiles

    @classmethod
    def from_assets(
        cls,
        sprite_sheet_path: Optional[Path] = None,
        tile_data_path: Optional[Path] = None,
    ) -> "TileAtlas":
        sprite_sheet_path = sprite_sheet_path or get_asset_path("images", "test_pipes.png")
        tile_data_path = tile_data_path or get_asset_path("data", "squares_data_unique.json")
        sprite_sheet = pygame.image.load(sprite_sheet_path).convert_alpha()
        tiles = cls._load_tile_metadata(tile_data_path)
        return cls(sprite_sheet=sprite_sheet, tiles=tiles)

    def _extract_tile(self, tile: TileMetadata) -> pygame.Surface:
        surface = pygame.Surface((tile.sprite_width, tile.sprite_height), pygame.SRCALPHA)
        surface.blit(self._sprite_sheet, (0, 0), tile.rect)
        return surface

    @staticmethod
    def _load_tile_metadata(path: Path) -> List[TileMetadata]:
        payload = json.loads(path.read_text())
        tiles: List[TileMetadata] = []
        for entry in payload:
            tiles.append(
                TileMetadata(
                    filename=entry["filename"],
                    up=entry["up"],
                    left=entry["left"],
                    down=entry["down"],
                    right=entry["right"],
                    sprite_x=entry["sprite_x"],
                    sprite_y=entry["sprite_y"],
                    sprite_width=entry["sprite_width"],
                    sprite_height=entry["sprite_height"],
                )
            )
        return tiles

    def draw_grid(
        self,
        target: pygame.Surface,
        origin: Tuple[int, int],
        columns: int,
        rows: int,
        scale: float,
        start_index: int = 0,
        padding: int = 4,
    ) -> None:
        if not self._tiles:
            return
        tile_width = int(self._tiles[0].metadata.sprite_width * scale)
        tile_height = int(self._tiles[0].metadata.sprite_height * scale)
        x_origin, y_origin = origin
        total_tiles = columns * rows
        for index in range(total_tiles):
            tile = self._tiles[(start_index + index) % len(self._tiles)]
            tile_surface = pygame.transform.smoothscale(tile.surface, (tile_width, tile_height))
            column = index % columns
            row = index // columns
            x = x_origin + column * (tile_width + padding)
            y = y_origin + row * (tile_height + padding)
            target.blit(tile_surface, (x, y))
