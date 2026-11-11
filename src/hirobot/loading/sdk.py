"""Vendor SDK requirements, read from each driver directory's ``manifest.yaml``.

Every SDK directory under :mod:`hirobot.drivers` (``drivers/body/unitree/sdk2/``,
``drivers/end_effector/unitree/sdk2_handcmd/``, ...) ships a ``manifest.yaml`` next to
its driver::

    sdk:
      - distribution: unitree_sdk2py   # name pip / importlib.metadata knows it by
        module: unitree_sdk2py         # top-level import name (default: distribution)
        minimum: "1.0.1"               # older versions are refused
        install: pip install -e <clone of unitree_sdk2_python>
    verified:                          # one entry per model x SDK version run on hardware
      - model: Unitree G1 (29 DoF)
        sdk: {unitree_sdk2py: "1.0.1 (a035ade)"}
        date: 2026-11-04
    maintainers: []

A driver calls :func:`require_sdk` with its own package at the start of
``connect()``. It raises :class:`ImportError` when an SDK is missing or older
than ``minimum``, and warns (once per process) when the installed version is
newer than every version in ``verified``: newer SDKs usually work, but nobody
has run that combination on hardware. Behavior that differs between SDK builds
is detected in the driver by checking for the feature itself, not by comparing
versions: two builds of ``unitree_sdk2py`` both report 1.0.1.
"""

from __future__ import annotations

import functools
import importlib.metadata
import importlib.resources
import importlib.util
import logging
import re
import sys
import warnings
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml

logger = logging.getLogger(__name__)

MANIFEST = "manifest.yaml"


@dataclass(frozen=True)
class SdkRequirement:
    """One SDK a driver needs."""

    distribution: str
    module: str
    minimum: Optional[str] = None
    install: Optional[str] = None


@dataclass(frozen=True)
class Manifest:
    """A driver directory's ``manifest.yaml``."""

    package: str
    sdk: Tuple[SdkRequirement, ...] = ()
    verified: Tuple[Dict[str, Any], ...] = ()
    maintainers: Tuple[str, ...] = ()
    raw: Dict[str, Any] = field(default_factory=dict, compare=False, repr=False)

    def tested_versions(self, distribution: str) -> List[str]:
        """Versions of ``distribution`` named in ``verified`` (the leading version token)."""
        out = []
        for entry in self.verified:
            v = (entry.get("sdk") or {}).get(distribution)
            if v:
                out.append(str(v).split()[0])
        return out


@functools.lru_cache(maxsize=None)
def load_manifest(package: str) -> Manifest:
    """Read and validate ``manifest.yaml`` from a driver package (e.g. ``__package__``)."""
    text = importlib.resources.files(package).joinpath(MANIFEST).read_text()
    data = yaml.safe_load(text) or {}
    reqs = []
    for item in data.get("sdk") or []:
        if "distribution" not in item:
            raise ValueError(f"{package}/{MANIFEST}: every sdk entry needs a 'distribution'")
        reqs.append(SdkRequirement(
            distribution=item["distribution"],
            module=item.get("module", item["distribution"]),
            minimum=None if item.get("minimum") is None else str(item["minimum"]),
            install=item.get("install"),
        ))
    return Manifest(package=package, sdk=tuple(reqs),
                    verified=tuple(data.get("verified") or ()),
                    maintainers=tuple(data.get("maintainers") or ()), raw=data)


def parse_version(text: str) -> Tuple:
    """A sortable key for a version string (PEP 440 via ``packaging`` when installed)."""
    try:
        from packaging.version import InvalidVersion, Version
    except ImportError:  # pragma: no cover - packaging is nearly always present
        Version = None
    if Version is not None:
        try:
            return (0, Version(text))
        except InvalidVersion:
            pass
    # Fallback: the leading dotted release numbers ("1.0.1rc2" -> (1, 0, 1)).
    m = re.match(r"\d+(\.\d+)*", text.strip())
    return (1, tuple(int(p) for p in m.group(0).split("."))) if m else (2, text)


def installed_version(req: SdkRequirement) -> Optional[str]:
    """The installed version of ``req``, or ``None`` if it cannot be told.

    Raises:
        ImportError: If the SDK is not installed at all.
    """
    try:
        return importlib.metadata.version(req.distribution)
    except importlib.metadata.PackageNotFoundError:
        pass
    # Importable without package metadata (a bare source checkout on sys.path,
    # or a stand-in module in tests): present, version unknown.
    if req.module in sys.modules:
        return None
    try:
        found = importlib.util.find_spec(req.module) is not None
    except (ImportError, ValueError):
        found = False
    if found:
        return None
    hint = f" Install it: {req.install}" if req.install else ""
    raise ImportError(f"{req.distribution} is not installed.{hint}")


_warned: set = set()


def require_sdk(package: str, who: Optional[str] = None) -> Dict[str, Optional[str]]:
    """Check the SDKs a driver package's manifest lists; return ``{distribution: version}``.

    Args:
        package: The driver's package, normally ``__package__``.
        who: Name used in messages (default: ``package``).

    Raises:
        ImportError: An SDK is missing, or older than its ``minimum``.
    """
    manifest = load_manifest(package)
    who = who or package
    found: Dict[str, Optional[str]] = {}
    for req in manifest.sdk:
        try:
            version = installed_version(req)
        except ImportError as exc:
            raise ImportError(f"{who}: {exc}") from None
        found[req.distribution] = version
        if version is None:
            key = (package, req.distribution, "unknown")
            if key not in _warned:
                _warned.add(key)
                logger.info("%s: %s version unknown (no package metadata); not checked",
                            who, req.distribution)
            continue
        if req.minimum and parse_version(version) < parse_version(req.minimum):
            hint = f" Install a newer one: {req.install}" if req.install else ""
            raise ImportError(
                f"{who} needs {req.distribution} >= {req.minimum}, found {version}.{hint}")
        tested = manifest.tested_versions(req.distribution)
        if tested and parse_version(version) > max(parse_version(t) for t in tested):
            key = (package, req.distribution, version)
            if key not in _warned:
                _warned.add(key)
                warnings.warn(
                    f"{who}: {req.distribution} {version} is newer than any version run on "
                    f"hardware ({', '.join(sorted(set(tested)))}). It may work; if it does, "
                    f"add a 'verified' entry to {package.replace('.', '/')}/{MANIFEST}.",
                    stacklevel=2)
    return found
