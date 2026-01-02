#!/usr/bin/env python3
"""
Arduino communication interface using serial communication
"""

import serial
from pathlib import Path
from typing import Optional

from core.config import load_arduino_config


class ArduinoInterface:
    """Handles communication with Arduino board via serial port"""
    
    def __init__(self):
        """Initialize Arduino interface"""
        self.serial_conn: Optional[serial.Serial] = None
        self.config = load_arduino_config()
        self.connected = False
    
    def connect(self) -> bool:
        """Establish connection to Arduino"""
        if self.connected:
            return True
        
        try:
            serial_config = self.config['serial']
            self.serial_conn = serial.Serial(
                port=serial_config['port'],
                baudrate=serial_config['baudrate'],
                timeout=serial_config['timeout']
            )
            self.connected = True
            print(f"Connected to Arduino on {serial_config['port']}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"Unexpected error connecting to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close connection to Arduino"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.connected = False
            print("Disconnected from Arduino")
    
    def is_connected(self) -> bool:
        """Check if Arduino is connected"""
        return self.connected and self.serial_conn and self.serial_conn.is_open
    
    def read_data(self) -> Optional[str]:
        """Read data from Arduino"""
        if not self.is_connected():
            return None
        
        try:
            if self.serial_conn.in_waiting > 0:
                delimiter = self.config['protocol']['message_delimiter']
                data = self.serial_conn.readline().decode('utf-8').strip()
                
                if self.config['debug']['log_serial']:
                    print(f"Arduino RX: {data}")
                
                return data
        except serial.SerialException as e:
            print(f"Error reading from Arduino: {e}")
            self.connected = False
        except UnicodeDecodeError as e:
            print(f"Error decoding Arduino data: {e}")
        
        return None
    
    def write_data(self, data: str) -> bool:
        """Send data to Arduino"""
        if not self.is_connected():
            return False
        
        try:
            delimiter = self.config['protocol']['message_delimiter']
            message = data + delimiter
            self.serial_conn.write(message.encode('utf-8'))
            
            if self.config['debug']['log_serial']:
                print(f"Arduino TX: {data}")
            
            return True
        except serial.SerialException as e:
            print(f"Error writing to Arduino: {e}")
            self.connected = False
            return False
    
    def send_command(self, command: str, value: Optional[str] = None) -> bool:
        """Send a formatted command to Arduino"""
        if value:
            message = f"{command}:{value}"
        else:
            message = command
        return self.write_data(message)


if __name__ == "__main__":
    # Test Arduino connection
    arduino = ArduinoInterface()
    if arduino.connect():
        print("Testing Arduino communication...")
        arduino.write_data("TEST")
        data = arduino.read_data()
        if data:
            print(f"Received: {data}")
        arduino.disconnect()
    else:
        print("Could not connect to Arduino. Check configuration.")

