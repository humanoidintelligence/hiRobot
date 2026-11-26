"""Simulation models (MJCF / URDF) from the ``humanoidintelligence/hirobot-assets`` dataset.

The models are not shipped with hiRobot. :func:`fetch` downloads the ones asked for from
the Hugging Face dataset into :func:`assets_dir` (``~/.cache/hirobot/assets``), keeping the
dataset's layout, so the built-in configs can point at fixed paths such as
``~/.cache/hirobot/assets/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml``.
Nothing here touches the network on import or when a config loads; only :func:`fetch`
does, and it needs ``pip install 'hirobot[assets]'`` (``huggingface_hub``).

Each name in :data:`ASSETS` is a list of ``allow_patterns`` for ``snapshot_download``:
the model directories (patterns ending in ``/*``) first, then the source's ``LICENSE`` and
``SOURCE.md`` and the dataset's root ``README.md``. The unitree_ros models are named after
the robot (``unitree_g1``); the MuJoCo Menagerie ones are ``menagerie_<menagerie directory>``.
"""

from __future__ import annotations

import fnmatch
import os
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Union

REPO_ID = "humanoidintelligence/hirobot-assets"
#: Environment variable that names another dataset with the same layout (a mirror or fork).
REPO_ENV = "HIROBOT_ASSETS_REPO"
# A tag, not a branch, so a later upload never changes what a given hiRobot version
# downloads. Every copy of the dataset carries the same tags on the same files.
REVISION = "v2"

_DIR = "~/.cache/hirobot/assets"


def _unitree_ros(description: str) -> List[str]:
    return [f"unitree_ros/robots/{description}/*", "unitree_ros/LICENSE",
            "unitree_ros/SOURCE.md", "README.md"]


def _menagerie(directory: str) -> List[str]:
    # Each Menagerie directory carries its own LICENSE.
    return [f"menagerie/{directory}/*", "menagerie/SOURCE.md", "README.md"]


def _fourier(*models: str) -> List[str]:
    root = "fourier/Wiki-GRx-Models"
    return [*(f"{root}/{m}/*" for m in models), f"{root}/LICENSE",
            "fourier/README.md", "fourier/SOURCE.md", "README.md"]


def _vendor(root: str, package: str, *legal: str) -> List[str]:
    """One package directory under ``root``, which holds the LICENSE and SOURCE.md."""
    return [f"{root}/{package}/*", f"{root}/LICENSE", *(f"{root}/{f}" for f in legal),
            f"{root}/SOURCE.md", "README.md"]


_MENAGERIE = {
    "unitree_g1": "Unitree G1 (MuJoCo Menagerie)",
    "unitree_h1": "Unitree H1 (MuJoCo Menagerie)",
    "agilex_piper": "AgileX PiPER arm (MuJoCo Menagerie)",
    "aloha": "ALOHA bimanual ViperX arms (MuJoCo Menagerie)",
    "franka_fr3": "Franka FR3 arm (MuJoCo Menagerie)",
    "franka_fr3_v2": "Franka FR3 arm, v2 model (MuJoCo Menagerie)",
    "universal_robots_ur5e": "Universal Robots UR5e arm (MuJoCo Menagerie)",
    "trossen_wx250s": "Trossen WidowX 250 S arm (MuJoCo Menagerie)",
    "robotiq_2f85": "Robotiq 2F-85 gripper (MuJoCo Menagerie)",
    "robotiq_2f85_v4": "Robotiq 2F-85 gripper, v4 model (MuJoCo Menagerie)",
    "realsense_d435i": "Intel RealSense D435i camera (MuJoCo Menagerie)",
}

# name -> allow_patterns, in the order `hirobot assets list` prints them.
ASSETS: Dict[str, List[str]] = {
    "unitree_g1": _unitree_ros("g1_description"),
    "unitree_h1": _unitree_ros("h1_description"),
    "unitree_h1_2": _unitree_ros("h1_2_description"),
    # unitree_ros's H2 MJCF has closed-linkage knees and ankles; unitree_mujoco's has one
    # motor per joint, which the simulation needs.
    "unitree_h2": [*_unitree_ros("h2_description"), "unitree_mujoco/unitree_robots/h2/*",
                   "unitree_mujoco/LICENSE", "unitree_mujoco/SOURCE.md"],
    "unitree_dex3": _unitree_ros("dexterous_hand_description"),
    **{f"menagerie_{d}": _menagerie(d) for d in _MENAGERIE},
    "fourier_gr1": _fourier("GR1_legacy/GR1", "GRX/GR1"),
    "fourier_gr2": _fourier("GRX/GR2"),
    "inspire_rh56dfx": _vendor("inspire", "rh56dfx_description"),
    "camera_zed": _vendor("cameras/zed", "zed_description"),
    "camera_realsense": _vendor("cameras/realsense", "realsense2_description",
                                "RealSense Copyright"),
    "camera_orbbec": _vendor("cameras/orbbec", "orbbec_description", "NOTICE"),
}

# name -> one line on what it is (for `hirobot assets list`).
DESCRIPTIONS: Dict[str, str] = {
    "unitree_g1": "Unitree G1, with Dex3-1 and Inspire hands (unitree_ros, MJCF + URDF)",
    "unitree_h1": "Unitree H1, with hands (unitree_ros, MJCF + URDF)",
    "unitree_h1_2": "Unitree H1-2 (unitree_ros, MJCF + URDF)",
    "unitree_h2": "Unitree H2 (unitree_ros URDF + MJCF; unitree_mujoco MJCF)",
    "unitree_dex3": "Unitree Dex3-1 hand (unitree_ros dexterous_hand_description)",
    **{f"menagerie_{d}": text + ", MJCF" for d, text in _MENAGERIE.items()},
    "fourier_gr1": "Fourier GR1T1 and GR1T2 (Wiki-GRx-Models, URDF, GPL-3.0)",
    "fourier_gr2": "Fourier GR2 (Wiki-GRx-Models, URDF + MJCF, GPL-3.0)",
    "inspire_rh56dfx": "Inspire RH56DFX hands, left and right (URDF)",
    "camera_zed": "Stereolabs ZED cameras (zed-ros2-description, meshes + xacro)",
    "camera_realsense": "Intel RealSense cameras (realsense2_description, meshes + xacro)",
    "camera_orbbec": "Orbbec cameras (meshes + xacro)",
}


def assets_dir() -> Path:
    """``~/.cache/hirobot/assets``: where :func:`fetch` puts the models and the configs look."""
    return Path(_DIR).expanduser()


def repo_id() -> str:
    """The dataset :func:`fetch` downloads from: ``$HIROBOT_ASSETS_REPO``, else :data:`REPO_ID`."""
    return os.environ.get(REPO_ENV) or REPO_ID


def model_patterns(name: str) -> List[str]:
    """The patterns of ``name`` that select its model directories (not license files)."""
    return [p for p in ASSETS[name] if p.endswith("/*")]


def _check(names: Iterable[str]) -> List[str]:
    names = list(names)
    unknown = [n for n in names if n not in ASSETS]
    if unknown:
        raise ValueError(f"unknown asset(s) {unknown}; known: {', '.join(ASSETS)}")
    return names


def is_fetched(name: str) -> bool:
    """Whether model files of ``name`` are in :func:`assets_dir`."""
    (name,) = _check([name])
    root = assets_dir()
    for pattern in model_patterns(name):
        start = root / pattern[:-2]
        if not start.is_dir():
            continue
        for dirpath, _dirs, files in os.walk(start):
            if files:
                return True
    return False


def fetch(names: Union[str, Iterable[str]], *, token: Optional[str] = None) -> Path:
    """Download the named models into :func:`assets_dir` and return that directory.

    Files already there and unchanged are not downloaded again. ``token`` is a Hugging
    Face token (default: the one ``hf auth login`` saved, or ``$HF_TOKEN``).
    """
    names = _check([names] if isinstance(names, str) else names)
    if not names:
        raise ValueError(f"no asset named; known: {', '.join(ASSETS)}")
    try:
        from huggingface_hub import snapshot_download
    except ImportError as e:
        raise RuntimeError(
            "fetching assets needs huggingface_hub: pip install 'hirobot[assets]'") from e
    patterns: List[str] = []
    for name in names:
        patterns += [p for p in ASSETS[name] if p not in patterns]
    root = assets_dir()
    root.mkdir(parents=True, exist_ok=True)
    snapshot_download(repo_id=repo_id(), repo_type="dataset", revision=REVISION,
                      allow_patterns=patterns, local_dir=str(root), token=token)
    return root


def asset_of(path: Union[str, Path]) -> Optional[str]:
    """The asset name whose model files include ``path``, or ``None`` if none does."""
    try:
        rel = Path(path).expanduser().relative_to(assets_dir()).as_posix()
    except ValueError:
        return None
    for name in ASSETS:
        if any(fnmatch.fnmatch(rel, p) for p in model_patterns(name)):
            return name
    return None


def fetch_hint(path: Union[str, Path]) -> str:
    """What to tell someone whose model file ``path`` is missing."""
    name = asset_of(path)
    if name is None:
        return "See `hirobot assets list` for the models hiRobot can download."
    return f"Download it with `hirobot assets fetch {name}` (pip install 'hirobot[assets]')."
