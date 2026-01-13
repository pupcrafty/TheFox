"""Sensor adapter for Arduino and simulated inputs."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from core.config import ArduinoConfig
import serial


@dataclass
class SensorState:
    values: List[float]


class SensorHub:
    def __init__(self, config: ArduinoConfig) -> None:
        self._config = config
        self._serial = None
        self._fallback_values = [0.0 for _ in range(config.sensor_count)]

        if config.enabled:
            self._serial = serial.Serial(config.port, config.baud_rate, timeout=0.01)

    def poll(self) -> SensorState:
        if self._serial is None:
            return SensorState(values=list(self._fallback_values))

        line = self._serial.readline().decode("utf-8").strip()
        if not line:
            return SensorState(values=list(self._fallback_values))

        parts = [float(value) for value in line.split(",") if value]
        if len(parts) != self._config.sensor_count:
            return SensorState(values=list(self._fallback_values))

        return SensorState(values=parts)

    def shutdown(self) -> None:
        if self._serial is not None:
            self._serial.close()
