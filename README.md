# Raspberry Pi Touch Screen Project

A fun interactive visual program built with Pygame, designed to run on a Raspberry Pi with a touch screen interface. The project integrates Arduino peripherals for additional input capabilities.

## Project Overview

This project creates an engaging touch-based interactive application that combines:
- **Visual Interface**: Pygame-based graphics optimized for Raspberry Pi touch screens
- **Touch Input**: Full touch screen support for user interaction
- **Arduino Integration**: Peripheral input devices connected via Arduino board for additional sensor/button inputs

## Hardware Requirements

- Raspberry Pi (3B+, 4, or newer recommended)
- Touch screen display (compatible with Raspberry Pi)
- Arduino board (Uno, Nano, or compatible)
- USB cable for Arduino connection
- Power supply for Raspberry Pi

## Software Requirements

- Raspberry Pi OS (Raspberry Pi OS with Desktop recommended)
- Python 3.7 or higher
- Pygame library
- PySerial library for Arduino communication

## Project Structure

```
TheFox/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── config/                   # Configuration files
│   ├── app_config.json      # Application settings
│   └── arduino_config.json  # Arduino communication settings
├── src/                      # Main application source code
│   ├── main.py              # Entry point for the application
│   ├── ui/                  # User interface modules
│   │   ├── __init__.py
│   │   └── app.py           # Main Pygame application class
│   ├── hardware/            # Hardware interface modules
│   │   ├── __init__.py
│   │   └── arduino_interface.py  # Arduino communication module
│   ├── core/                # Core application modules
│   │   ├── __init__.py
│   │   └── config.py        # Configuration management
│   └── utils/               # Utility modules
│       ├── __init__.py
│       └── helpers.py       # Helper functions
├── arduino/                  # Arduino firmware
│   └── firmware.ino         # Arduino sketch
├── assets/                   # Media assets
│   ├── images/              # Image files
│   ├── sounds/              # Sound files
│   └── fonts/               # Font files
└── docs/                     # Documentation
    └── SETUP.md             # Setup and installation guide
```

## Installation

1. **Clone or download this repository**
   ```bash
   cd TheFox
   ```

2. **Install Python dependencies**
   ```bash
   pip3 install -r requirements.txt
   ```

3. **Install Pygame (if not already installed)**
   ```bash
   sudo apt-get update
   sudo apt-get install python3-pygame
   ```

4. **Upload Arduino firmware**
   - Open `arduino/firmware.ino` in Arduino IDE
   - Upload to your Arduino board
   - Note the COM port (will be needed for configuration)

5. **Configure the application**
   - Edit `config/app_config.json` for display settings
   - Edit `config/arduino_config.json` to set the correct COM port for your Arduino

## Running the Application

1. **Ensure Arduino is connected** via USB to the Raspberry Pi

2. **Run the main application**
   ```bash
   python3 src/main.py
   ```

3. **For fullscreen touch interface**, the application will automatically detect and use the touch screen

## Configuration

### Application Configuration (`config/app_config.json`)
- Screen resolution and fullscreen settings
- Touch sensitivity
- Application theme and colors

### Arduino Configuration (`config/arduino_config.json`)
- Serial port (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`)
- Baud rate (typically 9600 or 115200)
- Timeout settings

## Development

### Adding Features
- Main application logic: `src/ui/app.py`
- Arduino communication: `src/hardware/arduino_interface.py`
- Configuration: `src/core/config.py`
- Utility functions: `src/utils/helpers.py`

### Testing
- Test on desktop first with a mouse (simulating touch)
- Test Arduino connection separately before integrating

## Troubleshooting

### Touch Screen Not Working
- Check display configuration: `sudo raspi-config`
- Verify touch screen drivers are installed

### Arduino Not Detected
- Check USB connection
- Verify COM port in `config/arduino_config.json`
- Test connection: `ls /dev/tty*` to find Arduino device

### Performance Issues
- Reduce screen resolution if needed
- Optimize image/sound assets
- Close unnecessary background processes

## License

[Your License Here]

## Author

[Your Name Here]

