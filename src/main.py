"""Application entry point for TheFox."""
from __future__ import annotations

from pathlib import Path

from app.app import TheFoxApp
from core.config import load_app_config, load_arduino_config


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    app_config = load_app_config(root / "config" / "app_config.json")
    arduino_config = load_arduino_config(root / "config" / "arduino_config.json")

    app = TheFoxApp(app_config=app_config, arduino_config=arduino_config)
    app.run()


if __name__ == "__main__":
    main()
