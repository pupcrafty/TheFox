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


class TouchScreenApp:
    """Main application class handling pygame window and touch input"""
    
    def __init__(self):
        """Initialize the application"""
        # Load configuration
        self.config = load_all_config()
        
        # Initialize Pygame
        pygame.init()
        pygame.mouse.set_visible(not self.config['display']['fullscreen'])
        
        # Set up display
        display_config = self.config['display']
        if display_config['fullscreen']:
            self.screen = pygame.display.set_mode(
                (display_config['width'], display_config['height']),
                pygame.FULLSCREEN
            )
        else:
            self.screen = pygame.display.set_mode(
                (display_config['width'], display_config['height'])
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
        
        # Initialize fonts
        self.font_large = pygame.font.Font(None, 72)
        self.font_medium = pygame.font.Font(None, 48)
        self.font_small = pygame.font.Font(None, 36)
        
        print("Application initialized successfully")
    
    def handle_events(self):
        """Process all pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_touch(event.pos, True)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.handle_touch(event.pos, False)
            elif event.type == pygame.FINGERDOWN or event.type == pygame.FINGERUP:
                # Handle native touch events if available
                x = int(event.x * self.config['display']['width'])
                y = int(event.y * self.config['display']['height'])
                is_pressed = (event.type == pygame.FINGERDOWN)
                self.handle_touch((x, y), is_pressed)
    
    def handle_touch(self, position, is_pressed):
        """Handle touch input at given position"""
        if is_pressed:
            print(f"Touch detected at {position}")
            # Add your touch handling logic here
    
    def update(self):
        """Update application state"""
        # Read data from Arduino if connected
        if self.arduino.is_connected():
            data = self.arduino.read_data()
            if data:
                self.handle_arduino_data(data)
    
    def handle_arduino_data(self, data):
        """Process data received from Arduino"""
        print(f"Arduino data: {data}")
        # Add your Arduino data processing logic here
    
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
        
        # Update display
        pygame.display.flip()
    
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

