#!/usr/bin/env python3
"""
Main entry point for the Raspberry Pi Touch Screen Application
"""

import sys
from pathlib import Path

# Add src directory to path
src_path = Path(__file__).parent
sys.path.insert(0, str(src_path))

from ui.app import TouchScreenApp


def main():
    """Initialize and run the application"""
    try:
        app = TouchScreenApp()
        app.run()
    except KeyboardInterrupt:
        print("\nApplication interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

