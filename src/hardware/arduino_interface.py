"""Arduino serial interface utilities."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import serial

from core.config import ArduinoConfig


@dataclass
class ArduinoReading:
    values: List[float]


class ArduinoInterface:
    def __init__(self, config: ArduinoConfig) -> None:
        self._config = config
        self._serial = serial.Serial(config.port, config.baud_rate, timeout=0.01)

    def read(self) -> ArduinoReading:
        line = self._serial.readline().decode("utf-8").strip()
        if not line:
            return ArduinoReading(values=[0.0 for _ in range(self._config.sensor_count)])

        parts = [float(value) for value in line.split(",") if value]
        if len(parts) != self._config.sensor_count:
            return ArduinoReading(values=[0.0 for _ in range(self._config.sensor_count)])

        return ArduinoReading(values=parts)

    def close(self) -> None:
        self._serial.close()
