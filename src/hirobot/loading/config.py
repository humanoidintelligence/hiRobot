"""Config files: built-in names, ``defaults:`` composition, merge rules.

A config is a YAML mapping with ``embodiment``, ``body`` and optional
``safety`` / ``sim`` sections. hiRobot ships its configs inside the package
(``hirobot/configs/``); refer to one by name, or give a path to your own file::

    make_robot("unitree_g1")                      # hirobot/configs/robots/unitree_g1.yaml
    make_robot("body/unitree/h1")                 # hirobot/configs/body/unitree/h1.yaml
    make_robot("~/my_robot.yaml")                 # a file

A name is looked up as ``configs/<name>.yaml``, then ``configs/robots/<name>.yaml``.
Anything ending in ``.yaml`` / ``.yml``, or starting with ``/``, ``~`` or ``.``, is a path.

**Composition.** A config can list other configs under ``defaults:``. They are
loaded in order and merged, then the file itself is merged on top::

    # hirobot/configs/robots/unitree_g1_dex3.yaml
    defaults: [robots/unitree_g1, end_effector/unitree/dex3_1]
    embodiment: {name: unitree-g1-dex3}

Entries are resolved like any config reference, except that a relative path
is taken relative to the file that lists it. So a user file can build on a
built-in part (``defaults: [body/unitree/g1_29dof, ./my_gains.yaml]``).

**Merge rules.** Mappings merge key by key, recursively. The lists of named
objects (``joints``, ``end_effectors``, ``cameras``, ``microphones``,
``speakers``) merge by ``name``: an entry with a known name is merged into it,
a new name is appended. So a hand's config appends its finger joints after the
body's and fills in the body's ``left_hand`` end-effector. Every other value,
including other lists (gains, armature), is replaced whole.

A device's ``driver: {type: ...}`` merges key by key while ``type`` stays the
same (an override can change just the address), and is replaced whole when
``type`` changes: the old driver's options mean nothing to the new one. So a
simulated G1 can swap its microphone for ``{type: wav_mic, path: ...}`` without
the Unitree driver's options coming along.
"""

from __future__ import annotations

import copy
import importlib.resources
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import yaml

NAMED_LISTS = ("joints", "end_effectors", "cameras", "microphones", "speakers")
_SUFFIXES = (".yaml", ".yml")

ConfigRef = Union[str, "os.PathLike[str]"]


def config_root() -> Path:
    """Directory of the built-in configs (inside the installed package)."""
    return Path(str(importlib.resources.files("hirobot").joinpath("configs")))


def _is_path(ref: str) -> bool:
    return ref.endswith(_SUFFIXES) or ref.startswith(("/", "~", "."))


def builtin_path(name: str) -> Optional[Path]:
    """The built-in config called ``name``, or ``None``."""
    root = config_root()
    for candidate in (root / f"{name}.yaml", root / "robots" / f"{name}.yaml"):
        if candidate.is_file():
            return candidate
    return None


def resolve(ref: ConfigRef, base: Optional[Path] = None) -> Path:
    """The file a config reference names.

    Args:
        ref: A built-in name or a path.
        base: Directory relative paths are taken from (default: the working directory).

    Raises:
        FileNotFoundError: Nothing matches ``ref``.
    """
    text = os.fspath(ref)
    if isinstance(ref, os.PathLike) or _is_path(text):
        path = Path(text).expanduser()
        if not path.is_absolute() and base is not None:
            path = base / path
        if not path.is_file():
            raise FileNotFoundError(f"config file not found: {path}")
        return path
    found = builtin_path(text)
    if found is None:
        if Path(text).is_file():  # a bare relative filename without a suffix
            return Path(text)
        raise FileNotFoundError(
            f"no built-in config {text!r} (see `hirobot configs`), and no such file")
    return found


def merge(base: Any, override: Any, key: Optional[str] = None) -> Any:
    """``override`` merged over ``base`` by the rules in the module docstring (new object)."""
    if isinstance(base, dict) and isinstance(override, dict):
        if "type" in base and "type" in override and base["type"] != override["type"]:
            return copy.deepcopy(override)  # another driver: its options start fresh
        out = copy.deepcopy(base)
        for k, v in override.items():
            out[k] = merge(out[k], v, k) if k in out else copy.deepcopy(v)
        return out
    if key in NAMED_LISTS and isinstance(base, list) and isinstance(override, list):
        out = copy.deepcopy(base)
        index = {_item_name(item, key): i for i, item in enumerate(out)}
        for item in override:
            name = _item_name(item, key)
            if name in index:
                out[index[name]] = merge(out[index[name]], item)
            else:
                index[name] = len(out)
                out.append(copy.deepcopy(item))
        return out
    return copy.deepcopy(override)


def _item_name(item: Any, key: str) -> str:
    if not isinstance(item, dict) or "name" not in item:
        raise ValueError(f"every entry of {key!r} needs a 'name', got {item!r}")
    return item["name"]


def _load(path: Path, chain: List[Path]) -> Dict[str, Any]:
    real = path.resolve()
    if real in chain:
        cycle = " -> ".join(str(p) for p in [*chain, real])
        raise ValueError(f"config defaults form a cycle: {cycle}")
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: a config must be a mapping, got {type(data).__name__}")
    defaults = data.pop("defaults", None) or []
    if isinstance(defaults, str):
        defaults = [defaults]
    out: Dict[str, Any] = {}
    for entry in defaults:
        out = merge(out, _load(resolve(entry, base=path.parent), [*chain, real]))
    return merge(out, data)


def load_config(ref: Union[ConfigRef, Dict[str, Any]]) -> Dict[str, Any]:
    """Load a config by name or path and expand its ``defaults:`` (a dict is returned as a copy)."""
    if isinstance(ref, dict):
        return copy.deepcopy(ref)
    return _load(resolve(ref), [])


def available_configs(group: Optional[str] = None) -> List[str]:
    """Names of the built-in configs, e.g. ``robots/unitree_g1``; ``group`` limits to one directory."""
    root = config_root()
    top = root / group if group else root
    return sorted(p.relative_to(root).with_suffix("").as_posix() for p in top.rglob("*.yaml"))
