#!/usr/bin/env python3
"""
Main application class for the touch screen interface
"""

import pygame
import sys
from pathlib import Path

# Add parent directories to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.config import load_all_config
from hardware.arduino_interface import ArduinoInterface
from ui.stick_figure import StickFigure


class TouchScreenApp:
    """Main application class handling pygame window and touch input"""
    
    def __init__(self):
        """Initialize the application"""
        # Load configuration
        self.config = load_all_config()
        
        # Initialize Pygame
        pygame.init()
        
        # Show mouse cursor if available
        # Pygame will automatically show/hide based on mouse presence
        pygame.mouse.set_visible(True)
        
        # Get screen resolution and calculate 9:16 aspect ratio dimensions
        display_config = self.config['display']
        screen_width, screen_height = self._calculate_9_16_resolution()
        
        # Set up display
        if display_config['fullscreen']:
            self.screen = pygame.display.set_mode(
                (screen_width, screen_height),
                pygame.FULLSCREEN
            )
        else:
            self.screen = pygame.display.set_mode(
                (screen_width, screen_height)
            )
        
        pygame.display.set_caption("Raspberry Pi Touch App")
        
        # Initialize clock for FPS control
        self.clock = pygame.time.Clock()
        self.fps = display_config['fps']
        
        # Initialize Arduino interface
        self.arduino = ArduinoInterface()
        arduino_config = self.config.get('arduino', {})
        if arduino_config.get('serial', {}).get('auto_connect', True):
            self.arduino.connect()
        
        # Application state
        self.running = True
        self.theme = self.config['theme']
        
        # Close button state
        self.screen_clicked = False
        self.button_alpha = 255  # 0-255 for transparency
        self.last_click_time = 0
        self.fade_delay = 2000  # 2 seconds before starting to fade (in milliseconds)
        self.fade_speed = 2  # Alpha decrease per frame
        
        # Mouse tracking
        self.last_mouse_pos = None
        self.mouse_moved = False
        
        # Time tracking for physics updates
        self.last_update_time = pygame.time.get_ticks()
        
        # Initialize fonts
        self.font_large = pygame.font.Font(None, 72)
        self.font_medium = pygame.font.Font(None, 48)
        self.font_small = pygame.font.Font(None, 36)
        
        # Close button properties
        self.button_size = 60
        self.button_padding = 20
        
        # Initialize stick figure
        self.stick_figure = StickFigure(
            x=screen_width // 2,
            y=screen_height // 2 - 50,
            scale=1.5
        )
        self.stick_figure.set_color(tuple(self.theme['text_color']))
        
        print("Application initialized successfully")
    
    def _calculate_9_16_resolution(self):
        """
        Calculate resolution with 9:16 aspect ratio based on screen size.
        Returns (width, height) tuple maintaining 9:16 ratio.
        """
        # Get the screen resolution
        screen_info = pygame.display.Info()
        screen_width = screen_info.current_w
        screen_height = screen_info.current_h
        
        # Target aspect ratio: 9:16 (width:height)
        target_ratio = 9 / 16
        
        # Calculate dimensions that fit within screen while maintaining 9:16 ratio
        # Option 1: Fit to screen width
        width_from_width = screen_width
        height_from_width = int(width_from_width / target_ratio)
        
        # Option 2: Fit to screen height
        height_from_height = screen_height
        width_from_height = int(height_from_height * target_ratio)
        
        # Choose the option that fits best within the screen
        if height_from_width <= screen_height:
            # Width-based calculation fits
            final_width = width_from_width
            final_height = height_from_width
        else:
            # Height-based calculation fits
            final_width = width_from_height
            final_height = height_from_height
        
        print(f"Screen resolution: {screen_width}x{screen_height}")
        print(f"Calculated 9:16 resolution: {final_width}x{final_height}")
        
        return final_width, final_height
    
    def handle_events(self):
        """Process all pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
            elif event.type == pygame.MOUSEMOTION:
                # Show mouse cursor when mouse movement is detected
                if not pygame.mouse.get_visible():
                    pygame.mouse.set_visible(True)
                self.mouse_moved = True
                self.last_mouse_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Show mouse cursor on click
                if not pygame.mouse.get_visible():
                    pygame.mouse.set_visible(True)
                self.handle_touch(event.pos, True)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.handle_touch(event.pos, False)
            elif event.type == pygame.FINGERDOWN or event.type == pygame.FINGERUP:
                # Handle native touch events if available
                x = int(event.x * self.screen.get_width())
                y = int(event.y * self.screen.get_height())
                is_pressed = (event.type == pygame.FINGERDOWN)
                self.handle_touch((x, y), is_pressed)
    
    def handle_touch(self, position, is_pressed):
        """Handle touch input at given position"""
        if is_pressed:
            print(f"Touch detected at {position}")
            
            # Check if close button was clicked first
            if self.screen_clicked and self.is_button_clicked(position):
                self.running = False
                return
            
            # Mark screen as clicked to show close button
            if not self.screen_clicked:
                self.screen_clicked = True
                self.button_alpha = 255
                self.last_click_time = pygame.time.get_ticks()
            else:
                # Reset button visibility on any click
                self.button_alpha = 255
                self.last_click_time = pygame.time.get_ticks()
            
            # Toggle acceleration visualization on screen click
            self.stick_figure.set_show_accelerations(not self.stick_figure.show_accelerations)
    
    def update(self):
        """Update application state"""
        # Calculate delta time for physics updates
        current_time = pygame.time.get_ticks()
        dt = (current_time - self.last_update_time) / 1000.0  # Convert to seconds
        self.last_update_time = current_time
        
        # Update stick figure with acceleration-based movement
        self.stick_figure.update(dt)
        
        # Handle button fading
        if self.screen_clicked:
            time_since_click = current_time - self.last_click_time
            
            # Start fading after delay
            if time_since_click > self.fade_delay:
                self.button_alpha = max(0, self.button_alpha - self.fade_speed)
        
        # Read data from Arduino if connected
        if self.arduino.is_connected():
            data = self.arduino.read_data()
            if data:
                self.handle_arduino_data(data)
    
    def handle_arduino_data(self, data):
        """Process data received from Arduino"""
        print(f"Arduino data: {data}")
        # Add your Arduino data processing logic here
    
    def is_button_clicked(self, position):
        """Check if the close button was clicked"""
        if not self.screen_clicked or self.button_alpha <= 0:
            return False
        
        screen_width = self.screen.get_width()
        button_x = screen_width - self.button_size - self.button_padding
        button_y = self.button_padding
        
        button_rect = pygame.Rect(button_x, button_y, self.button_size, self.button_size)
        return button_rect.collidepoint(position)
    
    def get_button_rect(self):
        """Get the rectangle for the close button"""
        screen_width = self.screen.get_width()
        button_x = screen_width - self.button_size - self.button_padding
        button_y = self.button_padding
        return pygame.Rect(button_x, button_y, self.button_size, self.button_size)
    
    def draw(self):
        """Render the application"""
        # Clear screen with background color
        bg_color = self.theme['background_color']
        self.screen.fill(tuple(bg_color))
        
        # Draw title
        title_text = self.font_large.render(
            "Raspberry Pi Touch App",
            True,
            tuple(self.theme['text_color'])
        )
        title_rect = title_text.get_rect(center=(self.screen.get_width() // 2, 100))
        self.screen.blit(title_text, title_rect)
        
        # Draw Arduino connection status
        arduino_status = "Connected" if self.arduino.is_connected() else "Disconnected"
        status_color = (0, 255, 0) if self.arduino.is_connected() else (255, 0, 0)
        status_text = self.font_medium.render(
            f"Arduino: {arduino_status}",
            True,
            status_color
        )
        status_rect = status_text.get_rect(center=(self.screen.get_width() // 2, 200))
        self.screen.blit(status_text, status_rect)
        
        # Draw instructions
        instructions = [
            "Touch the screen to interact",
            "Press ESC to exit"
        ]
        y_offset = 300
        for instruction in instructions:
            inst_text = self.font_small.render(instruction, True, tuple(self.theme['text_color']))
            inst_rect = inst_text.get_rect(center=(self.screen.get_width() // 2, y_offset))
            self.screen.blit(inst_text, inst_rect)
            y_offset += 50
        
        # Draw FPS if enabled
        if self.config['debug']['show_fps']:
            fps_text = self.font_small.render(
                f"FPS: {int(self.clock.get_fps())}",
                True,
                tuple(self.theme['text_color'])
            )
            self.screen.blit(fps_text, (10, 10))
        
        # Draw stick figure
        self.stick_figure.draw(self.screen)
        
        # Draw close button if screen has been clicked and button is visible
        if self.screen_clicked and self.button_alpha > 0:
            self.draw_close_button()
        
        # Update display
        pygame.display.flip()
    
    def draw_close_button(self):
        """Draw the close button in the top-right corner"""
        button_rect = self.get_button_rect()
        
        # Create a surface for the button with alpha support
        button_surface = pygame.Surface((self.button_size, self.button_size), pygame.SRCALPHA)
        
        # Draw button background (red circle/rounded rectangle)
        button_color = (200, 50, 50, self.button_alpha)  # Red with alpha
        pygame.draw.rect(button_surface, button_color, (0, 0, self.button_size, self.button_size), border_radius=10)
        
        # Draw X symbol
        line_color = (255, 255, 255, self.button_alpha)  # White with alpha
        line_width = 4
        margin = 15
        # Draw X: two diagonal lines
        pygame.draw.line(
            button_surface, 
            line_color, 
            (margin, margin), 
            (self.button_size - margin, self.button_size - margin), 
            line_width
        )
        pygame.draw.line(
            button_surface, 
            line_color, 
            (self.button_size - margin, margin), 
            (margin, self.button_size - margin), 
            line_width
        )
        
        # Blit the button surface to the main screen
        self.screen.blit(button_surface, button_rect)
    
    def run(self):
        """Main application loop"""
        print("Starting application...")
        while self.running:
            self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(self.fps)
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        if self.arduino.is_connected():
            self.arduino.disconnect()
        pygame.quit()


if __name__ == "__main__":
    app = TouchScreenApp()
    app.run()




