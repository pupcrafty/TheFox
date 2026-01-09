# Setup Guide

## Prerequisites

### Raspberry Pi Setup
1. Install Raspberry Pi OS (preferably with desktop environment)
2. Update system packages:
   ```bash
   sudo apt-get update
   sudo apt-get upgrade
   ```

### Touch Screen Setup
1. Connect your touch screen to the Raspberry Pi
2. Configure display (if needed):
   ```bash
   sudo raspi-config
   ```
   - Navigate to "Display Options"
   - Configure resolution and orientation as needed

### Arduino Setup
1. Install Arduino IDE on your computer (not Raspberry Pi)
2. Connect Arduino to computer via USB
3. Upload the firmware from `arduino/firmware.ino`
4. Disconnect Arduino from computer
5. Connect Arduino to Raspberry Pi via USB

## Installation Steps

### 1. Clone/Download Project
Navigate to your desired location and clone or extract the project.

### 2. Install Python Dependencies
```bash
cd TheFox
pip3 install -r requirements.txt
```

If you encounter permission errors, use:
```bash
pip3 install --user -r requirements.txt
```

### 3. Install Pygame
Pygame can be installed via pip or apt:
```bash
sudo apt-get install python3-pygame
```
or
```bash
pip3 install pygame
```

### 4. Install PySerial
```bash
sudo apt-get install python3-serial
```

### 5. Configure Arduino Connection

Find your Arduino's serial port:
```bash
ls /dev/tty* | grep -i acm
# or
ls /dev/tty* | grep -i usb
```

Common ports:
- `/dev/ttyACM0` - Most Arduino boards
- `/dev/ttyUSB0` - USB-to-serial adapters

Edit `config/arduino_config.json` and update the port:
```json
{
  "serial": {
    "port": "/dev/ttyACM0",  // Change this to your port
    ...
  }
}
```

### 6. Set Permissions (if needed)

If you get permission errors accessing the serial port:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in for changes to take effect.

### 7. Test Arduino Connection

You can test the Arduino connection separately:
```bash
python3 src/arduino_interface.py
```

### 8. Run the Application

```bash
python3 src/main.py
```

## Troubleshooting

### Touch Screen Not Responding
- Verify touch screen is properly connected
- Check if touch drivers are installed:
  ```bash
  lsmod | grep touch
  ```
- Test touch with: `xinput test-xi2 --root`

### Arduino Not Detected
- Verify USB connection
- Check if Arduino appears in device list:
  ```bash
  lsusb
  ```
- Check if serial port exists:
  ```bash
  ls -l /dev/ttyACM*
  ```
- Verify baud rate matches in both Arduino code and config

### Permission Denied on Serial Port
- Add user to dialout group (see step 6 above)
- Or run with sudo (not recommended):
  ```bash
  sudo python3 src/main.py
  ```

### Application Won't Start
- Check Python version: `python3 --version` (should be 3.7+)
- Verify all dependencies are installed: `pip3 list`
- Check for error messages in terminal

## Running on Startup

To make the application start automatically on boot:

1. Create a service file:
   ```bash
   sudo nano /etc/systemd/system/touchscreen-app.service
   ```

2. Add the following (adjust paths as needed):
   ```ini
   [Unit]
   Description=Raspberry Pi Touch Screen App
   After=graphical.target

   [Service]
   Type=simple
   User=pi
   WorkingDirectory=/home/pi/TheFox
   ExecStart=/usr/bin/python3 /home/pi/TheFox/src/main.py
   Restart=always

   [Install]
   WantedBy=graphical.target
   ```

3. Enable and start the service:
   ```bash
   sudo systemctl enable touchscreen-app.service
   sudo systemctl start touchscreen-app.service
   ```

## Development Tips

- Test on desktop first with mouse (simulates touch)
- Use SSH to access Raspberry Pi remotely during development
- Enable debug options in config files for troubleshooting
- Use `print()` statements or Python logging for debugging




