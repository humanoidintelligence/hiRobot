"""Every driver directory with a manifest has its own CODEOWNERS line, in src/ and tests/."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _owned_paths():
    paths = set()
    for line in (ROOT / ".github" / "CODEOWNERS").read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            path, *owners = line.split()
            assert owners, f"CODEOWNERS line without an owner: {line!r}"
            paths.add(path.strip("/"))
    return paths


def test_every_driver_directory_has_owners():
    owned = _owned_paths()
    drivers = ROOT / "src" / "hirobot" / "drivers"
    missing = []
    for manifest in sorted(drivers.rglob("manifest.yaml")):
        rel = manifest.parent.relative_to(drivers).as_posix()
        for prefix in ("src/hirobot/drivers", "tests/drivers"):
            if f"{prefix}/{rel}" not in owned:
                missing.append(f"/{prefix}/{rel}/")
    assert not missing, f"add these to .github/CODEOWNERS: {missing}"
