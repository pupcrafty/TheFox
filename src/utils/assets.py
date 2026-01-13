"""Helpers for locating asset files."""
from __future__ import annotations

from pathlib import Path


def get_asset_path(*parts: str) -> Path:
    """Return a Path to an asset within the repository."""
    root = Path(__file__).resolve().parents[2]
    return root / "assets" / Path(*parts)
