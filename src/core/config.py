#!/usr/bin/env python3
"""
Configuration management module
"""

import json
from pathlib import Path
from typing import Dict, Any


def load_app_config() -> Dict[str, Any]:
    """Load application configuration from JSON file"""
    base_path = Path(__file__).parent.parent.parent
    config_path = base_path / "config" / "app_config.json"
    
    with open(config_path, 'r') as f:
        return json.load(f)


def load_arduino_config() -> Dict[str, Any]:
    """Load Arduino configuration from JSON file"""
    base_path = Path(__file__).parent.parent.parent
    config_path = base_path / "config" / "arduino_config.json"
    
    with open(config_path, 'r') as f:
        return json.load(f)


def load_stick_figure_config() -> Dict[str, Any]:
    """Load stick figure physics configuration from JSON file"""
    base_path = Path(__file__).parent.parent.parent
    config_path = base_path / "config" / "stick_figure_config.json"
    
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Warning: stick_figure_config.json not found at {config_path}, using defaults")
        return {}


def load_all_config() -> Dict[str, Any]:
    """Load all configuration files into a single dictionary"""
    config = load_app_config()
    config['arduino'] = load_arduino_config()
    config['stick_figure'] = load_stick_figure_config()
    return config




