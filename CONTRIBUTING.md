# Contributing to hiRobot

hiRobot is one interface for many robots. Most contributions add a robot model,
a driver for a new SDK, or a hardware report for an SDK version. This page says
what each kind of change needs so it can be reviewed quickly.

## Set up

```bash
git clone https://github.com/humanoidintelligence/hiRobot.git && cd hiRobot
pip install -e '.[dev]'          # add '.[sim]' for the MuJoCo driver and twin
python -m pytest                 # run from the repo root; no GPU, robot or SDK needed
ruff check .                     # what CI enforces (rules in pyproject.toml)
```

Every vendor SDK is faked in the tests, so the full suite runs on any machine.
CI (`.github/workflows/ci.yml`) runs it on Python 3.9-3.13 with the core install,
with all extras on 3.9 and 3.13, on macOS, and with the oldest numpy / pydantic /
pyyaml that `pyproject.toml` allows; it also builds the wheel and loads every
built-in config from it. Tests that need an extra skip without it
(`pytest.importorskip`), so a new one should too.

## What a change needs

| Change | Add | Don't touch |
|---|---|---|
| New model on a supported SDK (e.g. another Unitree humanoid) | `src/hirobot/configs/body/<vendor>/<model>.yaml`, a `robots/<name>.yaml` that composes it with `defaults:`, a row in the README's support table | Driver code |
| Supported SDK, new version, ran on hardware | One `verified` entry in the driver's `manifest.yaml` (model, SDK version, date), a row in the setup guide's version table | `driver.py`, unless behavior changed |
| New SDK or protocol | A new directory `src/hirobot/drivers/<part>/<vendor>/<sdk>/` with `driver.py` and `manifest.yaml`; `tests/drivers/<part>/<vendor>/<sdk>/{fake_sdk,test_contract}.py`; CODEOWNERS lines; a setup guide or section | Existing drivers |
| Core (`types.py`, `robot.py`, `safety.py`, `config.py`, ...) | Open an issue first. Every driver depends on these | |

The README's [Add a new robot](README.md#add-a-new-robot) has the full rules and
naming. In short:

- Code is named after the SDK, not the product: `drivers/body/unitree/sdk2/`
  covers G1, H1, H1-2 and H2.
- Directories under `drivers/` and `configs/` use the same singular part names:
  `body`, `end_effector`, `camera`, `audio`. Simulators sit in `drivers/sim/<simulator>/`,
  vendor-neutral drivers in `drivers/<part>/generic/<driver>/`.
- Register with the part's decorator: `register_body`, `register_end_effector`,
  `register_camera`, `register_audio` (all exported from `hirobot`).
- Registered names are `<vendor>_<SDK or protocol>` (`unitree_sdk2`); simulators
  use the simulator's name (`toy`, `mujoco`). A released name never changes meaning. An incompatible SDK generation gets a new
  directory and a new name.
- Import the SDK inside `connect()`, after `require_sdk(__package__)`, so
  `import hirobot` works without it.
- Override `_send_action`, never `send_action`: the safety gate runs in
  `send_action`.
- Report `base_orientation` as `[x, y, z, w]` and joint vectors in
  `Embodiment.joints` order.
- Keep health signals (battery, temperatures) in `get_status()`, not in
  `Observation`.

## Tests

- A new driver passes the shared contract checks in `tests/drivers/contract.py`
  (`check_robot`, `check_sim`, `check_hand` / `check_opening` for an end-effector on
  its `joints` / `opening` interface, `check_camera`, `check_microphone`,
  `check_speaker`) against a fake of the SDK calls it makes.
  `tests/drivers/test_coverage.py` fails until it does.
- Test what your driver does beyond the contract (joint maps, gain handling,
  vendor quirks) next to it in `test_driver.py`.
- A change to a built-in config also changes `tests/data/expanded_configs.json`,
  the expanded form of every whole-robot config. Regenerate it, then check the
  diff is what you meant:

  ```bash
  python -c "import json; from hirobot.config import available_configs, load_config; \
  names = available_configs('robots'); \
  print(json.dumps({n: load_config(n) for n in names}, indent=1, sort_keys=True))" \
  > tests/data/expanded_configs.json
  ```
- CI cannot run hardware. Say in the pull request what you ran on a real robot,
  and record it as a `verified` entry.

## Working on real robots

A pull request that changes what a driver sends to the hardware must say how it
was checked. For a first run, the setup guides in `docs/setup/` use this order:
read-only, then a shadow rehearsal in MuJoCo where the driver supports it, then
a hung or supported robot with low gains, with the e-stop in reach. Don't put
real IP addresses, serial numbers or calibration from your lab into shared
configs; keep them in a local config that uses `defaults:` to build on the
built-in one.

## Pull requests

- One topic per pull request. Keep refactors separate from behavior changes.
- CI passes. Beyond the rules CI enforces, add no new `ruff check` findings in
  the files you touched.
- Update the docs your change makes wrong: README tables, `docs/api.md`, the
  setup guide.
- Commit messages: a short summary line, then what changed and why.
- `.github/CODEOWNERS` assigns reviewers by path. When you add a driver or take
  one over, add your GitHub handle there and under `maintainers:` in its
  `manifest.yaml`.

## Reporting problems

Open a GitHub issue with the robot model, the driver name, the SDK version
(`pip show <sdk>`), the hiRobot commit, and the full error. For a hardware
problem, say what the robot did.

## License

hiRobot is licensed under [Apache-2.0](LICENSE). By contributing you agree that
your contribution is licensed under the same terms.
