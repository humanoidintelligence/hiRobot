# Example configs

Configs you copy and edit. They are not installed with the package: load them
by path, and give `defaults:` built-in names (`hirobot configs` lists them).

```bash
hirobot info --config examples/humanoid.yaml
```

```python
from hirobot import make_robot

robot = make_robot("examples/humanoid.yaml")
```

| File | Shows |
|---|---|
| `humanoid.yaml`, `arm.yaml` | A robot written from scratch: `assembly:` with the devices inline, no `defaults:` |
| `unitree_g1_real.yaml` | A built-in robot on hardware: `defaults:` plus the body's driver and network card |
| `unitree_g1_read_only.yaml` | Read the robot without commanding it: the driver's `read_only` |
| `rehearse_in_sim.py` | Rehearse a motion in MuJoCo from the real robot's current pose; the real robot is only read |
| `unitree_h1_inspire_camera.yaml` | Change one part (the head camera's address and size) with `overrides` |
| `franka_ur5e_dual_real.yaml` | A built-in robot of two robots on hardware: `overrides` per arm |
| `ur5e_dual_real.yaml` | Two UR5e arms, one config for real and `sim=True`: `overrides` per arm, a `sim:` section |
| `ur5e_dual_checked.yaml` | The same arms with the `sim_check` guarding each command from a private MuJoCo world |
| `ur5e_dual_inline.yaml` | The three ways to give a part: `from:` a reference, an inline `assembly:` node, an inline `device:` |
| `lab_two_cells.yaml` | Nodes inside nodes: a lab of two dual-arm cells, with per-part names and `overrides` down the tree |
| `g1_ur5e_cell.yaml` | A workcell mixing a G1, a UR5e and a camera; the G1 placed with `origin:` as a moving part |
| `mobile_ur5e.yaml` | A UR5e on a mobile base (a floating node); uses `devices/mobile_base.yaml` |
| `devices/mobile_base.yaml` | An example device file referenced by `mobile_ur5e.yaml` (not a robot on its own) |

Values in `<...>` are placeholders for your setup. The `unitree_*` files are
explained in [docs/setup/unitree.md](../docs/setup/unitree.md).
