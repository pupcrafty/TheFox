# TheFox

An interactive particle-physics and pseudo-3D corridor experience designed for Raspberry Pi touch screens with Arduino sensor input. This repository now focuses on a fresh architecture built directly from the project goals.

## Vision

TheFox blends a fluid particle simulation (SPH-inspired) with a scrolling 3D corridor rendered as a decagon tunnel. The experience is tuned for 9:16 portrait displays, real-time sensor interaction, and long-running installations.

## Key Goals

- **Fluid-like particle system** with up to 2,200 particles, SPH-inspired forces, and metaball-style visuals.
- **3D corridor illusion** using decagonal segments and perspective scaling.
- **Arduino sensors** mapped to four corner emitters for responsive control.
- **Touch-first UI** optimized for Raspberry Pi portrait displays.

## Project Structure

```
TheFox/
├── config/
│   ├── app_config.json
│   ├── arduino_config.json
│   └── force_sequences.json
├── docs/
│   ├── PROJECT_GOALS.md
│   └── SETUP.md
├── src/
│   ├── app/                # Application orchestration
│   ├── core/               # Configuration models
│   ├── hardware/           # Arduino helpers
│   ├── input/              # Touch + sensor adapters
│   ├── physics/            # SPH-inspired particle system
│   ├── rendering/          # Corridor + particle rendering
│   ├── utils/              # Utilities (timing, math)
│   └── main.py             # Entry point
└── requirements.txt
```

## Getting Started

1. Install dependencies:
   ```bash
   pip3 install -r requirements.txt
   ```

2. Configure settings in `config/app_config.json` and `config/arduino_config.json`.

3. Run the application:
   ```bash
   python3 src/main.py
   ```

## Next Milestones

- Replace placeholder particle integration with full SPH + spatial hashing.
- Add metaball shader-based rendering for the particle blob.
- Implement corridor tile sprite sheets and texture mapping.
- Expand sensor mapping and force sequence playback.

For detailed intent and success criteria, see `docs/PROJECT_GOALS.md`.
