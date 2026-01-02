#!/usr/bin/env python3
"""
Helper utility functions
"""

import os
from pathlib import Path


def get_project_root() -> Path:
    """Get the project root directory"""
    return Path(__file__).parent.parent.parent


def get_asset_path(asset_type: str, filename: str) -> Path:
    """
    Get the full path to an asset file
    
    Args:
        asset_type: Type of asset ('images', 'sounds', 'fonts')
        filename: Name of the asset file
    
    Returns:
        Path object to the asset file
    """
    root = get_project_root()
    return root / "assets" / asset_type / filename


def ensure_directory_exists(path: Path):
    """Ensure a directory exists, create if it doesn't"""
    path.mkdir(parents=True, exist_ok=True)


def clamp(value, min_value, max_value):
    """Clamp a value between min and max"""
    return max(min_value, min(value, max_value))


def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another"""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

