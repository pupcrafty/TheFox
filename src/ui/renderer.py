#!/usr/bin/env python3
"""
Renderer class for handling layered drawing operations
Collects all assets that need to be drawn and renders them in correct order
"""

import pygame
from typing import Callable, List, Tuple, Optional, Any
from enum import IntEnum


class RenderLayer(IntEnum):
    """Layer ordering for rendering (lower = drawn first, appears behind)"""
    BACKGROUND = 0
    WIREFRAME = 10
    PARTICLES = 20
    HEART_IMAGE = 30
    EMITTER_INDICATORS = 40
    UI_TEXT = 50
    CLOSE_BUTTON = 60


class Renderer:
    """
    Renderer class that collects drawing operations and executes them in layer order
    Only writes to screen once at the end
    """
    
    def __init__(self, screen: pygame.Surface):
        """
        Initialize the renderer
        
        Args:
            screen: The pygame surface to render to
        """
        self.screen = screen
        self.draw_operations: List[Tuple[int, Callable]] = []
    
    def clear(self, color: Tuple[int, int, int]):
        """
        Clear the screen with the given color
        
        Args:
            color: RGB color tuple
        """
        self.screen.fill(color)
    
    def add_draw_operation(self, layer: int, draw_func: Callable[[pygame.Surface], None]):
        """
        Add a drawing operation to the render queue
        
        Args:
            layer: Layer number (lower = drawn first/behind)
            draw_func: Function that takes a surface and draws to it
        """
        self.draw_operations.append((layer, draw_func))
    
    def add_surface(self, layer: int, surface: pygame.Surface, position: Tuple[int, int]):
        """
        Add a surface blit operation to the render queue
        
        Args:
            layer: Layer number
            surface: Surface to blit
            position: (x, y) position to blit at
        """
        def draw_func(screen):
            screen.blit(surface, position)
        self.add_draw_operation(layer, draw_func)
    
    def add_surface_rect(self, layer: int, surface: pygame.Surface, rect: pygame.Rect):
        """
        Add a surface blit operation with a rect to the render queue
        
        Args:
            layer: Layer number
            surface: Surface to blit
            rect: Rectangle specifying position
        """
        def draw_func(screen):
            screen.blit(surface, rect)
        self.add_draw_operation(layer, draw_func)
    
    def render(self):
        """
        Execute all drawing operations in layer order and flip the display
        This is the only method that should write to the screen
        """
        # Sort operations by layer (lower layers drawn first)
        self.draw_operations.sort(key=lambda x: x[0])
        
        # Execute all drawing operations
        for layer, draw_func in self.draw_operations:
            draw_func(self.screen)
        
        # Clear operations for next frame
        self.draw_operations.clear()
        
        # Update display (single flip)
        pygame.display.flip()
    
    def render_to_surface(self) -> pygame.Surface:
        """
        Execute all drawing operations but return the surface instead of flipping
        Useful for rendering to an intermediate surface
        
        Returns:
            The rendered surface
        """
        # Sort operations by layer
        self.draw_operations.sort(key=lambda x: x[0])
        
        # Execute all drawing operations
        for layer, draw_func in self.draw_operations:
            draw_func(self.screen)
        
        # Clear operations
        self.draw_operations.clear()
        
        return self.screen
