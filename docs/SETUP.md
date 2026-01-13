# Setup Guide

## Requirements

- Raspberry Pi OS (Desktop recommended)
- Python 3.9+
- Touch screen display (portrait 9:16)
- Arduino with sensor inputs (optional for development)

## Install Dependencies

```bash
pip3 install -r requirements.txt
```

## Configure the App

- `config/app_config.json` controls rendering, physics, and corridor settings.
- `config/arduino_config.json` enables/disables Arduino input and port settings.
- `config/force_sequences.json` contains demo force patterns for scripted tests.

## Run the App

```bash
python3 src/main.py
```

## Arduino Notes

If `enabled` is true in `config/arduino_config.json`, ensure the Arduino is connected
and streaming comma-separated sensor values (one value per sensor).
