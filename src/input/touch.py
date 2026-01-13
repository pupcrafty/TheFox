"""Touch input tracking."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import pygame


@dataclass
class TouchState:
    positions: List[Tuple[int, int]] = field(default_factory=list)


class TouchInput:
    def __init__(self) -> None:
        self.state = TouchState()

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.state.positions.append(event.pos)
        elif event.type == pygame.MOUSEMOTION and event.buttons[0]:
            self.state.positions.append(event.pos)
        elif event.type == pygame.MOUSEBUTTONUP:
            self.state.positions.clear()
