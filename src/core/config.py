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


def load_all_config() -> Dict[str, Any]:
    """Load all configuration files into a single dictionary"""
    config = load_app_config()
    config['arduino'] = load_arduino_config()
    return config

