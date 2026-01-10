# Stick Figure Physics Demo (Pygame + Pymunk)

This is a **stand-alone visual demo** implementing the constraints from the spec:

- Forces can ONLY be applied to **four sensor points**:
  - `left_upper_arm`, `right_upper_arm`, `left_thigh`, `right_thigh`
- The torso is **NOT** a valid force target.
- Torso **tilt** is constrained (default ±45°) and protected from flipping.
- A simulated **in-place 180° spin** is achieved via coordinated sensor forces.
  - Note: true "yaw" rotation is not representable in a pure 2D rigid-body sim,
    so this demo tracks a separate `yaw` value driven by the *pattern of sensor forces*.
- Limb crossing is discouraged by **side-normalization forces** (left stays left, right stays right).
- JSON force sequences can be played for testing.

## Run

```bash
pip install -r requirements.txt
python main.py
```

## Controls

- **SPACE**: Play `demo_spin` sequence
- **J**: Play `demo_jump` sequence
- **R**: Reset figure
- **ESC / close window**: Quit

## Files

- `main.py` — Pygame loop + rendering + input
- `src/forces.py` — JSON sequences + force fade + force manager
- `src/stick_figure.py` — physical rig + constraint enforcement + yaw indicator
- `config/force_sequences.json` — sample sequences

