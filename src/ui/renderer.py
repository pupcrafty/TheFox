#!/usr/bin/env python3
"""
Renderer class for handling layered drawing operations
Collects all assets that need to be drawn and renders them in correct order
"""

import ctypes
import pygame
from array import array
from typing import Callable, List, Tuple
from enum import IntEnum
from OpenGL.GL import (
    glBindTexture,
    glBindBuffer,
    glBufferData,
    glClear,
    glClearColor,
    glDeleteProgram,
    glDeleteTextures,
    glDeleteBuffers,
    glDrawArrays,
    glEnableVertexAttribArray,
    glGenBuffers,
    glGenTextures,
    glGetAttribLocation,
    glGetUniformLocation,
    glLinkProgram,
    glShaderSource,
    glTexImage2D,
    glTexParameteri,
    glTexSubImage2D,
    glUniform1i,
    glUseProgram,
    glVertexAttribPointer,
    glViewport,
    glCreateProgram,
    glCreateShader,
    glCompileShader,
    glAttachShader,
    glDeleteShader,
    glGetShaderiv,
    glGetShaderInfoLog,
    glGetProgramiv,
    glGetProgramInfoLog,
    GL_ARRAY_BUFFER,
    GL_COLOR_BUFFER_BIT,
    GL_COMPILE_STATUS,
    GL_FLOAT,
    GL_FRAGMENT_SHADER,
    GL_LINEAR,
    GL_LINK_STATUS,
    GL_RGBA,
    GL_STATIC_DRAW,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TRIANGLE_STRIP,
    GL_UNSIGNED_BYTE,
)


class RenderLayer(IntEnum):
    """Layer ordering for rendering (lower = drawn first, appears behind)"""
    BACKGROUND = 0
    TILES = 5
    WIREFRAME = 10
    PARTICLES = 20
    HEART_IMAGE = 30
    EMITTER_INDICATORS = 40
    UI_TEXT = 50
    CLOSE_BUTTON = 60


class Renderer:
    """OpenGL-backed renderer that composites a software surface and presents via GL."""

    def __init__(self, screen_size: Tuple[int, int], render_surface: pygame.Surface):
        """Initialize the renderer.

        Args:
            screen_size: (width, height) of the display
            render_surface: Software surface to draw onto before uploading to GL
        """
        self.screen_width, self.screen_height = screen_size
        self.render_surface = render_surface
        self.draw_operations: List[Tuple[int, Callable]] = []
        self._program = None
        self._vbo = None
        self._texture = None
        self._position_loc = None
        self._texcoord_loc = None
        self._texture_loc = None
        self._init_gl()

    def _compile_shader(self, shader_type: int, source: str) -> int:
        shader = glCreateShader(shader_type)
        glShaderSource(shader, source)
        glCompileShader(shader)
        status = glGetShaderiv(shader, GL_COMPILE_STATUS)
        if not status:
            log = glGetShaderInfoLog(shader).decode("utf-8")
            glDeleteShader(shader)
            raise RuntimeError(f"Shader compile failed: {log}")
        return shader

    def _init_gl(self) -> None:
        glViewport(0, 0, self.screen_width, self.screen_height)
        vertex_source = """
        attribute vec2 a_position;
        attribute vec2 a_texcoord;
        varying vec2 v_texcoord;
        void main() {
            v_texcoord = a_texcoord;
            gl_Position = vec4(a_position, 0.0, 1.0);
        }
        """
        fragment_source = """
        precision mediump float;
        varying vec2 v_texcoord;
        uniform sampler2D u_texture;
        void main() {
            gl_FragColor = texture2D(u_texture, v_texcoord);
        }
        """
        vertex_shader = self._compile_shader(GL_VERTEX_SHADER, vertex_source)
        fragment_shader = self._compile_shader(GL_FRAGMENT_SHADER, fragment_source)
        program = glCreateProgram()
        glAttachShader(program, vertex_shader)
        glAttachShader(program, fragment_shader)
        glLinkProgram(program)
        status = glGetProgramiv(program, GL_LINK_STATUS)
        if not status:
            log = glGetProgramInfoLog(program).decode("utf-8")
            glDeleteProgram(program)
            raise RuntimeError(f"Program link failed: {log}")
        glDeleteShader(vertex_shader)
        glDeleteShader(fragment_shader)
        self._program = program
        self._position_loc = glGetAttribLocation(program, "a_position")
        self._texcoord_loc = glGetAttribLocation(program, "a_texcoord")
        self._texture_loc = glGetUniformLocation(program, "u_texture")

        quad_vertices = [
            -1.0, -1.0, 0.0, 0.0,
             1.0, -1.0, 1.0, 0.0,
            -1.0,  1.0, 0.0, 1.0,
             1.0,  1.0, 1.0, 1.0,
        ]
        vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(
            GL_ARRAY_BUFFER,
            len(quad_vertices) * 4,
            array("f", quad_vertices),
            GL_STATIC_DRAW,
        )
        self._vbo = vbo

        texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            self.screen_width,
            self.screen_height,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            None,
        )
        self._texture = texture
    
    def clear(self, color: Tuple[int, int, int]):
        """
        Clear the screen with the given color
        
        Args:
            color: RGB color tuple
        """
        self.render_surface.fill(color)
    
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
            draw_func(self.render_surface)
        
        # Clear operations for next frame
        self.draw_operations.clear()
        
        # Update display (single flip)
        self._present()

    def _present(self) -> None:
        raw = pygame.image.tostring(self.render_surface, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            self.screen_width,
            self.screen_height,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            raw,
        )
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT)
        glUseProgram(self._program)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glEnableVertexAttribArray(self._position_loc)
        glVertexAttribPointer(self._position_loc, 2, GL_FLOAT, False, 16, ctypes.c_void_p(0))
        glEnableVertexAttribArray(self._texcoord_loc)
        glVertexAttribPointer(self._texcoord_loc, 2, GL_FLOAT, False, 16, ctypes.c_void_p(8))
        glUniform1i(self._texture_loc, 0)
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
        pygame.display.flip()

    def shutdown(self) -> None:
        if self._program is not None:
            glDeleteProgram(self._program)
        if self._texture is not None:
            glDeleteTextures([self._texture])
        if self._vbo is not None:
            glDeleteBuffers(1, [self._vbo])
