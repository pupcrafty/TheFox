#!/usr/bin/env python3
"""
Tile manager for handling hallway tiles
Manages tiles placed on hallway wall surfaces with 3D perspective projection
Generates tiles directly in screen space like the wireframe
"""

import pygame
import math
import random
import string
from typing import List, Dict, Tuple, Optional, Callable
import numpy as np


def generate_decagon_vertices(center_x, center_y, radius):
    """
    Generate vertices for a regular decagon (10-sided polygon)
    Returns list of (x, y) tuples in screen space
    """
    vertices = []
    num_sides = 10
    for i in range(num_sides):
        angle = (2 * math.pi * i) / num_sides - math.pi / 2  # Start at top (-90 degrees)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    return vertices


def get_perspective_scale(z, perspective_fov):
    """Get scale factor for object at depth z"""
    # Safety check: prevent division by zero
    denominator = perspective_fov + z
    if abs(denominator) < 1e-6:  # Very small or zero
        return 1.0  # Default scale
    return perspective_fov / denominator


class Tile:
    """Represents a single tile on a hallway surface"""
    
    def __init__(self, vertices: List[Tuple[float, float]], letter: str = None, side_index: int = 0, depth_index: int = 0, sprite_surface: Optional[pygame.Surface] = None):
        """
        Initialize a tile
        
        Args:
            vertices: List of (x, y) vertices defining the tile shape in SCREEN SPACE
            letter: Optional letter to display on tile (deprecated, kept for compatibility)
            side_index: Index of the decagon side (0-9) for deterministic sprite selection
            depth_index: Index along depth for deterministic sprite selection
            sprite_surface: Optional pygame Surface containing the sprite tile to render
        """
        self.vertices = vertices  # Screen space vertices (already projected)
        self.sprite_surface = sprite_surface  # Sprite surface to render
        # Keep letter for backwards compatibility, but we'll use sprites instead
        if letter is not None:
            self.letter = letter
        else:
            # Generate deterministic letter based on position (side_index and depth_index)
            # This ensures the same tile position always gets the same letter across frames
            alphabet_len = len(string.ascii_uppercase)
            if alphabet_len > 0:
                letter_index = (side_index * 17 + depth_index * 31) % alphabet_len
                self.letter = string.ascii_uppercase[letter_index]
            else:
                self.letter = 'A'  # Fallback if alphabet is somehow empty


class TileManager:
    """Manages tiles placed on hallway wall surfaces"""
    
    def __init__(self, tile_size: float = 40.0, tile_color: Tuple[int, int, int] = (128, 128, 128), 
                 sprite_surfaces: Optional[List[pygame.Surface]] = None):
        """
        Initialize the tile manager
        
        Args:
            tile_size: Base size of tiles in screen space
            tile_color: RGB color tuple for tiles (grey by default, used as fallback)
            sprite_surfaces: Optional list of pygame Surfaces containing sprite tiles to use
        """
        self.tile_size = tile_size
        self.tile_color = tile_color
        self.tiles: List[Tile] = []
        self.sprite_surfaces = sprite_surfaces or []  # List of sprite surfaces to randomly choose from
        
    def update_tiles_for_segments(self, hallway_segments: List[Dict], 
                                  screen_width: int, screen_height: int,
                                  perspective_fov: float, vanishing_point_y: float,
                                  generate_floor: bool = False):
        """
        Update tiles based on hallway segments
        Generates tiles directly in screen space like the wireframe
        
        Args:
            hallway_segments: List of segment dictionaries with keys:
                - z_depth: Z position (depth)
                - base_radius: Base radius of decagon at near plane
                - segment_length: Length of segment in Z
            screen_width: Screen width in pixels
            screen_height: Screen height in pixels
            perspective_fov: Field of view for perspective projection
            vanishing_point_y: Y position of vanishing point (0.0-1.0)
            generate_floor: If True, also generate floor tiles (not implemented yet)
        """
        if len(hallway_segments) < 2:
            self.tiles = []
            return
        
        # Sort segments by Z depth (front to back)
        sorted_segments = sorted(hallway_segments, key=lambda s: s["z_depth"])
        
        # Clear existing tiles
        self.tiles = []
        
        # Generate tiles for wall surfaces between adjacent segments
        for i in range(len(sorted_segments) - 1):
            segment_front = sorted_segments[i]
            segment_back = sorted_segments[i + 1]
            
            # Create wall surfaces between these two decagon rings
            wall_tiles = self._create_wall_tiles(
                segment_front, segment_back,
                screen_width, screen_height,
                perspective_fov, vanishing_point_y
            )
            self.tiles.extend(wall_tiles)
        
        if generate_floor:
            # TODO: Generate floor tiles if needed
            pass
    
    def _create_wall_tiles(self, segment_front: Dict, segment_back: Dict,
                          screen_width: int, screen_height: int,
                          perspective_fov: float, vanishing_point_y: float) -> List[Tile]:
        """
        Create tiles for the wall surface between two segments
        Generates tiles directly in screen space like the wireframe
        
        Args:
            segment_front: Front segment (closer to camera, smaller Z)
            segment_back: Back segment (farther from camera, larger Z)
            screen_width: Screen width in pixels
            screen_height: Screen height in pixels
            perspective_fov: Field of view for perspective projection
            vanishing_point_y: Y position of vanishing point (0.0-1.0)
            
        Returns:
            List of Tile objects with screen-space vertices
        """
        tiles = []
        
        # Get Z depths for both segments
        z_front = segment_front["z_depth"]
        z_back = segment_back["z_depth"]
        
        # Calculate perspective scale for both segments (like wireframe does)
        scale_front = get_perspective_scale(z_front, perspective_fov)
        scale_back = get_perspective_scale(z_back, perspective_fov)
        
        # Calculate scaled radii in screen space (like wireframe does)
        radius_front = segment_front["base_radius"] * scale_front
        radius_back = segment_back["base_radius"] * scale_back
        
        # Skip if radii are too small to be visible
        if radius_front < 1.0 and radius_back < 1.0:
            return tiles
        
        # Center of screen (vanishing point) - same as wireframe
        center_x = screen_width / 2
        center_y = screen_height * vanishing_point_y
        
        # Generate decagon vertices for both rings in SCREEN SPACE (like wireframe)
        num_sides = 10
        vertices_front = generate_decagon_vertices(center_x, center_y, radius_front)
        vertices_back = generate_decagon_vertices(center_x, center_y, radius_back)
        
        # Create tiles for each wall segment (between adjacent vertices)
        for i in range(num_sides):
            # Get vertices for this wall segment
            v1_front = vertices_front[i]
            v2_front = vertices_front[(i + 1) % num_sides]
            v1_back = vertices_back[i]
            v2_back = vertices_back[(i + 1) % num_sides]
            
            # ONE TILE PER SIDE - tiles will align across rings
            # Calculate depth of wall segment (difference in Z)
            wall_depth = z_back - z_front
            
            # Safety check: skip if wall_depth is invalid or tile_size is zero
            if wall_depth <= 0 or self.tile_size <= 0:
                continue
            
            # Number of tiles along depth (vertical along the wall)
            # Use average scale to determine screen-space depth for consistent alignment
            avg_scale = (scale_front + scale_back) / 2.0
            scaled_tile_size = self.tile_size * avg_scale
            
            # Calculate number of tiles, with safety check to prevent division by zero
            if self.tile_size > 0:
                num_tiles_along_depth = max(1, int(wall_depth / self.tile_size))
            else:
                num_tiles_along_depth = 1
            
            # Create exactly ONE tile per side (j=0 only), spanning the full side length
            # This ensures tiles align consistently across rings
            for k in range(num_tiles_along_depth):
                # Tile spans the full segment (0.0 to 1.0)
                t_segment_start = 0.0
                t_segment_end = 1.0
                
                # Calculate depth interpolation for this tile
                # Safety check: prevent division by zero
                if num_tiles_along_depth > 0:
                    t_depth_start = k / num_tiles_along_depth
                    t_depth_end = (k + 1) / num_tiles_along_depth
                else:
                    t_depth_start = 0.0
                    t_depth_end = 1.0
                
                # Interpolate tile vertices in SCREEN SPACE
                # Front-left vertex (at front ring, segment start)
                v1_front_tile = (
                    v1_front[0] * (1 - t_segment_start) + v2_front[0] * t_segment_start,
                    v1_front[1] * (1 - t_segment_start) + v2_front[1] * t_segment_start
                )
                # Back-left vertex (at back ring, segment start)
                v1_back_tile = (
                    v1_back[0] * (1 - t_segment_start) + v2_back[0] * t_segment_start,
                    v1_back[1] * (1 - t_segment_start) + v2_back[1] * t_segment_start
                )
                # Front-right vertex (at front ring, segment end)
                v2_front_tile = (
                    v1_front[0] * (1 - t_segment_end) + v2_front[0] * t_segment_end,
                    v1_front[1] * (1 - t_segment_end) + v2_front[1] * t_segment_end
                )
                # Back-right vertex (at back ring, segment end)
                v2_back_tile = (
                    v1_back[0] * (1 - t_segment_end) + v2_back[0] * t_segment_end,
                    v1_back[1] * (1 - t_segment_end) + v2_back[1] * t_segment_end
                )
                
                # Create tile vertices (quadrilateral) in SCREEN SPACE
                tile_vertices = [
                    v1_front_tile,  # Front-left (at front ring, segment start)
                    v2_front_tile,  # Front-right (at front ring, segment end)
                    v2_back_tile,   # Back-right (at back ring, segment end)
                    v1_back_tile    # Back-left (at back ring, segment start)
                ]
                
                # Create tile with screen-space vertices (no projection needed)
                # Select a random sprite for this tile (deterministic based on position for consistency)
                sprite_surface = None
                if self.sprite_surfaces:
                    # Use deterministic random selection based on position for consistency
                    random.seed(i * 1000 + k)  # Deterministic seed based on position
                    sprite_index = random.randint(0, len(self.sprite_surfaces) - 1)
                    sprite_surface = self.sprite_surfaces[sprite_index]
                    random.seed()  # Reset random seed
                
                # Pass side_index (i) and depth_index (k) for deterministic sprite generation
                tile = Tile(tile_vertices, side_index=i, depth_index=k, sprite_surface=sprite_surface)
                tiles.append(tile)
        
        return tiles
    
    def get_draw_function(self, screen_width: int, screen_height: int,
                         perspective_fov: float, vanishing_point_y: float,
                         near_plane_z: float, far_plane_z: float,
                         font: Optional[pygame.font.Font] = None) -> Callable:
        """
        Get a draw function for rendering tiles
        
        Args:
            screen_width: Screen width in pixels
            screen_height: Screen height in pixels
            perspective_fov: Field of view for perspective projection (unused now, kept for compatibility)
            vanishing_point_y: Y position of vanishing point (0.0-1.0) (unused now, kept for compatibility)
            near_plane_z: Z distance of near plane (unused now, kept for compatibility)
            far_plane_z: Z distance of far plane (unused now, kept for compatibility)
            font: Optional font for rendering letters. If None, a default font is used
            
        Returns:
            A function that takes a screen surface and draws all tiles
        """
        # Use provided font or create default
        if font is None:
            try:
                tile_font = pygame.font.Font(None, 24)
            except:
                tile_font = pygame.font.SysFont('arial', 16)
        else:
            tile_font = font
        
        def draw_tiles(screen: pygame.Surface):
            """Draw all tiles to the screen"""
            # Tiles are already in screen space, no sorting by Z needed
            # Just draw them all
            
            sprite_draw_count = 0
            for tile in self.tiles:
                # Skip if tile is too small or has invalid vertices
                if len(tile.vertices) < 3:
                    continue
                
                # Convert vertices to integers for pygame drawing
                screen_vertices_int = [(int(v[0]), int(v[1])) for v in tile.vertices]
                
                # Check if tile is visible (at least partially on screen)
                min_x = min(v[0] for v in screen_vertices_int)
                max_x = max(v[0] for v in screen_vertices_int)
                min_y = min(v[1] for v in screen_vertices_int)
                max_y = max(v[1] for v in screen_vertices_int)
                
                # Skip if completely off screen
                if max_x < 0 or min_x > screen_width or max_y < 0 or min_y > screen_height:
                    continue
                
                # Draw sprite tile or fallback to colored polygon
                if tile.sprite_surface is not None:
                    # Draw sprite with perspective transform (trapezoidal skew)
                    self._draw_perspective_sprite(screen, tile.sprite_surface, screen_vertices_int)
                    sprite_draw_count += 1
                else:
                    # Fallback: draw tile as filled polygon (grey)
                    pygame.draw.polygon(screen, self.tile_color, screen_vertices_int)
                    
                    # Draw border (darker grey)
                    pygame.draw.polygon(screen, (100, 100, 100), screen_vertices_int, 1)
                    
                    # Draw letter in center of tile (white) if no sprite
                    if len(screen_vertices_int) > 0:
                        center_x = sum(v[0] for v in screen_vertices_int) / len(screen_vertices_int)
                        center_y = sum(v[1] for v in screen_vertices_int) / len(screen_vertices_int)
                        
                        # Render letter text
                        letter_surface = tile_font.render(tile.letter, True, (255, 255, 255))
                        letter_rect = letter_surface.get_rect(center=(center_x, center_y))
                        
                        # Only draw letter if tile is large enough
                        tile_size_estimate = max(max_x - min_x, max_y - min_y)
                        if tile_size_estimate > 15:  # Only draw letter if tile is reasonably sized
                            screen.blit(letter_surface, letter_rect)
        
        return draw_tiles
    
    def _draw_perspective_sprite(self, screen: pygame.Surface, sprite: pygame.Surface, vertices: List[Tuple[int, int]]):
        """
        Draw a sprite with perspective transform to fit trapezoidal vertices
        Uses a simpler approach: scale sprite and use polygon clipping for trapezoid shape
        
        Args:
            screen: Surface to draw to
            sprite: Sprite surface (rectangular)
            vertices: List of 4 (x, y) vertices forming a trapezoid in screen space
                      Order: [top-left, top-right, bottom-right, bottom-left]
        """
        if len(vertices) != 4:
            # Fallback to simple polygon draw
            pygame.draw.polygon(screen, self.tile_color, vertices)
            return
        
        # Extract vertices
        tl = vertices[0]  # top-left
        tr = vertices[1]  # top-right
        br = vertices[2]  # bottom-right
        bl = vertices[3]  # bottom-left
        
        # Calculate bounding box
        min_x = min(v[0] for v in vertices)
        max_x = max(v[0] for v in vertices)
        min_y = min(v[1] for v in vertices)
        max_y = max(v[1] for v in vertices)
        
        # Get sprite dimensions
        sprite_w, sprite_h = sprite.get_size()
        if sprite_w <= 0 or sprite_h <= 0:
            return
        
        # Calculate bounding box size
        output_w = max(1, int(max_x - min_x))
        output_h = max(1, int(max_y - min_y))
        
        # OPTIMIZATION: For performance, use simple scaling without masking
        # Creating masks and multiple surfaces per tile is too expensive
        # Just scale the sprite to the bounding box - it will look rectangular but perform much better
        # TODO: Implement proper perspective transform with caching if needed
        
        # Simple scaling approach (fast)
        scaled_sprite = pygame.transform.scale(sprite, (output_w, output_h))
        screen.blit(scaled_sprite, (int(min_x), int(min_y)))
