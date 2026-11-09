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
| `humanoid.yaml`, `arm.yaml` | A config written from scratch: `embodiment:` and `body:`, no `defaults:` |
| `unitree_g1_real.yaml` | A built-in robot on hardware: `defaults:` plus the driver and network card |
| `unitree_g1_read_only.yaml` | Read the robot without commanding it: `read_only` |
| `unitree_g1_shadow.yaml` | Rehearse in a MuJoCo twin from the robot's current pose: `sim: shadow` |
| `unitree_h1_inspire_camera.yaml` | Change a camera's address and size by its name |
| `franka_ur5e_dual_real.yaml` | A built-in group on hardware: `overrides` per member |

Values in `<...>` are placeholders for your setup. The `unitree_*` files are
explained in [docs/setup/unitree.md](../docs/setup/unitree.md).
