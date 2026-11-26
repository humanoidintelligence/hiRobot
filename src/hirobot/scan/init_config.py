"""``hirobot init``: write a local config for the robot ``hirobot scan`` sees.

The written file inherits a built-in robot config (``defaults: [robots/<model>]``)
and only sets what is local to this machine: the real body driver and the
network card on every part whose driver takes one. Gains and safety limits stay
the built-in defaults. After writing, :func:`verify_config` opens the robot
read-only (the path ``hirobot view`` uses), reads one observation and reports
which parts sent data. Nothing here sends a command, holds or resets the robot.

v1 writes Unitree configs only (the body driver ``unitree_sdk2``).
"""

from __future__ import annotations

import builtins
import datetime
import importlib
import inspect
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np
import yaml

#: Keys an ``--answers`` file may set.
ANSWER_KEYS = ("model", "interface", "overwrite", "verify")
#: The body driver v1 writes.
UNITREE_BODY_DRIVER = "unitree_sdk2"
#: Device kinds that only receive data, so they are safe to open next to a read-only body.
_RECEIVE_ONLY = ("camera", "microphone")
#: Candidate confidence above which a non-interactive run takes the model without asking.
_CERTAIN = 0.9
#: Drivers that take ``network_interface`` but ignore it (ZeroMQ routes by address).
_NIC_IGNORED = ("unitree_image_server",)


# ------------------------------------------------------------------ models


def _model_name(model: str) -> str:
    """``unitree_g1`` / ``robots/unitree_g1`` / ``robots/unitree_g1.yaml`` -> ``robots/unitree_g1``."""
    name = str(model).strip().removesuffix(".yaml")
    return name if name.startswith("robots/") else f"robots/{name}"


def _device_driver(dev: Any) -> Dict[str, Any]:
    """The device file's own ``driver:`` (before the robot config's overrides)."""
    from hirobot.loading.config import load_config

    if dev.source is not None:
        drv = load_config(str(dev.source)).get("driver")
        if isinstance(drv, str):
            drv = {"type": drv}
        if drv:
            return dict(drv)
    return dict(dev.driver or {})


def check_model(model: str) -> str:
    """The built-in name of ``model``; raise ``ValueError`` unless v1 can write a config for it."""
    from hirobot.loading.build import only_body
    from hirobot.loading.config import available_configs
    from hirobot.loading.tree import load_robot

    name = _model_name(model)
    if name not in available_configs("robots"):
        raise ValueError(f"unknown model {model!r}; Unitree models: {', '.join(unitree_models())}")
    spec = load_robot(name)
    if spec.sim is not None:
        raise ValueError(f"{name} is a simulation config; pick the hardware one")
    _, body = only_body(spec, "hirobot init")
    if _device_driver(body).get("type") != UNITREE_BODY_DRIVER:
        raise ValueError(f"{name} is not a Unitree robot; hirobot init writes Unitree configs "
                         f"only ({', '.join(unitree_models())})")
    return name


def unitree_models() -> List[str]:
    """Built-in robot configs whose body uses the Unitree SDK2 driver (no simulation configs)."""
    from hirobot.loading.config import available_configs
    from hirobot.loading.tree import load_robot

    out = []
    for name in available_configs("robots"):
        if not name.startswith("robots/unitree_"):
            continue
        try:
            spec = load_robot(name)
            bodies = [d for _, d in spec.root.walk() if getattr(d, "kind", None) == "body"]
        except Exception:  # noqa: BLE001 - a broken built-in must not hide the others
            continue
        if spec.sim is None and len(bodies) == 1 and \
                _device_driver(bodies[0]).get("type") == UNITREE_BODY_DRIVER:
            out.append(name)
    return out


def _driver_class(kind: str, type_name: str) -> Any:
    from hirobot.devices.audio import get_audio
    from hirobot.devices.camera import get_camera
    from hirobot.devices.end_effector import get_end_effector
    from hirobot.loading.registry import get_body

    get = {"body": get_body, "end_effector": get_end_effector, "camera": get_camera,
           "microphone": get_audio, "speaker": get_audio}[kind]
    try:
        return get(type_name)
    except Exception:  # noqa: BLE001 - an unknown driver just gets no NIC
        return None


def _takes(cls: Any, arg: str) -> bool:
    return cls is not None and arg in inspect.signature(cls.__init__).parameters


# ------------------------------------------------------------------ render


def _one_line(value: Any) -> str:
    """``value`` as text safe inside a YAML comment line."""
    return " ".join(str(value).split())


def _flow(value: Mapping[str, Any]) -> str:
    return yaml.safe_dump(dict(value), default_flow_style=True, sort_keys=False,
                          width=1000).strip()


def _seen_lines(interface: str, report: Mapping[str, Any]) -> List[str]:
    uni = dict((report or {}).get("unitree") or {})
    lines = [f"Seen by `hirobot scan` on {interface}: status {uni.get('status') or 'unknown'}"]
    if uni.get("msg_family"):
        lines.append(f"  message family {uni['msg_family']}, "
                     f"{uni.get('motors_reporting')} of {uni.get('motor_slots')} motor slots "
                     f"reporting, mode_machine {uni.get('mode_machine')}")
    service = uni.get("motion_service") or {}
    if service.get("checked"):
        mode = service.get("mode")
        lines.append(f"  built-in motion service: {mode!r}" if mode else
                     "  built-in motion service: none active")
    hands = {k: v for k, v in (uni.get("hands") or {}).items()}
    if hands:
        lines.append("  hands: " + ", ".join(
            f"{k} {'seen' if v else 'not seen, please confirm'}" for k, v in hands.items()))
    for note in uni.get("notes") or []:
        lines.append(f"  note: {note}")
    if uni.get("error"):
        lines.append(f"  error: {uni['error']}")
    return [_one_line(line) if not line.startswith("  ") else "  " + _one_line(line)
            for line in lines]


def part_overrides(model: str, interface: str) -> Dict[str, Dict[str, Any]]:
    """Part path -> the ``driver:`` override the local config writes for it."""
    from hirobot.loading.tree import DeviceSpec, load_robot

    out: Dict[str, Dict[str, Any]] = {}
    for path, dev in load_robot(model).root.walk():
        if not isinstance(dev, DeviceSpec):
            continue
        own = _device_driver(dev)
        if not own.get("type"):
            continue
        if dev.kind == "body":
            drv = {"type": own["type"]}
            if "msg_family" in own:
                drv["msg_family"] = own["msg_family"]  # go for the H1, hg otherwise
            drv["network_interface"] = interface
            out[path] = drv
        elif own["type"] not in _NIC_IGNORED and \
                _takes(_driver_class(dev.kind, own["type"]), "network_interface"):
            out[path] = {"network_interface": interface}
    return out


def render_config(model: str, interface: str, report: dict) -> str:
    """The YAML text of the local config for ``model`` on ``interface``."""
    model = _model_name(model)
    if not interface or any(c.isspace() for c in str(interface)):
        raise ValueError(f"bad network interface {interface!r}")
    today = datetime.date.today().isoformat()
    header = [f"{model.split('/')[-1]} on {interface}, written by `hirobot init` on {today}.",
              ""]
    header += _seen_lines(interface, report)
    header += ["",
               f"Inherits {model}; this file only picks the real drivers and the network card.",
               "Gains and safety limits are the built-in defaults.",
               "Check it before the first commanded motion: see docs/setup/unitree.md."]
    lines = [("# " + h).rstrip() for h in header]
    lines += ["", f"defaults: [{model}]", "", "assembly:", "  parts:"]
    for path, drv in part_overrides(model, interface).items():
        head, *rest = path.split("/")
        lines.append(f"    {head}:")
        lines.append("      overrides:")
        indent = "        "
        for name in rest:
            lines.append(f"{indent}{name}:")
            indent += "  "
        lines.append(f"{indent}driver: {_flow(drv)}")
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------ verify


def _not_read_only(spec: Any) -> List[Dict[str, Any]]:
    """Parts left out of the read-only check: drivers that could change the robot.

    Parts that only receive (cameras, microphones) stay; an end-effector is
    made read-only with the body; any other driver without a ``read_only``
    argument (a speaker sets its volume on connect) is left out.
    """
    from hirobot.loading.tree import DeviceSpec

    skipped: List[Dict[str, Any]] = []
    for p, dev in spec.root.walk():
        if not isinstance(dev, DeviceSpec) or not dev.driver or dev.kind == "body":
            continue
        if dev.kind in _RECEIVE_ONLY:
            continue
        if _takes(_driver_class(dev.kind, dev.driver.get("type", "")), "read_only"):
            continue
        skipped.append({"path": p, "kind": dev.kind, "status": "skipped",
                        "note": f"driver {dev.driver.get('type')!r} has no read_only mode; "
                                "not opened"})
    return skipped


def open_read_only(path: Union[str, os.PathLike], drop: Sequence[str] = ()) -> Any:
    """``make_robot`` with the body driver ``read_only: true`` and the parts in ``drop`` left out.

    The same checks as ``hirobot view`` (:func:`hirobot.sim.view.open_read_only`):
    one body, not simulated, and a body driver with a ``read_only`` argument.
    End-effector drivers become read-only with the body.
    """
    # What the opened robot puts on the bus: no ChannelPublisher and no command
    # topic (rt/lowcmd, rt/dex3/*/cmd). SDK RPC clients for read-only queries
    # (CheckMode, the videohub camera's GetImageSample) open their own request
    # channel. Drivers that would write (the speaker's SetVolume on open) are
    # in ``drop``.
    from hirobot import make_robot
    from hirobot.loading.build import only_body
    from hirobot.loading.registry import get_body
    from hirobot.loading.tree import load_robot

    body_path, dev = only_body(load_robot(str(path)), "hirobot init")
    name = (dev.driver or {"type": "toy"})["type"]
    cls = get_body(name)
    if cls.simulated:
        raise ValueError(f"{path} drives a simulated robot (body driver {name})")
    if not _takes(cls, "read_only"):
        raise ValueError(f"body driver {name!r} has no read_only mode, so it is not opened "
                         "(it could command the robot)")
    over: Dict[str, Any] = {p: {"driver": None} for p in drop}
    over[body_path] = {"driver": dict(dev.driver or {}, read_only=True)}
    return make_robot(str(path), overrides=over)


def _tag_connect_failures(robot: Any, tags: List[Tuple[str, BaseException]]) -> None:
    """Wrap each device's connect (``connect`` / ``start`` / ``open``) to record which one raised.

    The wrappers live on these driver instances only, which are thrown away
    after the check. The body's connect is tagged ``""``: its failure is never retried.
    """
    groups = [("connect", getattr(robot, "end_effectors", {}) or {}),
              ("start", getattr(robot, "cameras", {}) or {}),
              ("open", getattr(robot, "microphones", {}) or {}),
              ("open", getattr(robot, "speakers", {}) or {})]
    body = getattr(robot, "body", None)
    if body is not None:
        groups.append(("connect", {"": body}))
    for method, devices in groups:
        for name, dev in devices.items():
            original = getattr(dev, method, None)
            if original is None:
                continue

            def wrapped(*args, _orig=original, _name=name, **kwargs):
                try:
                    return _orig(*args, **kwargs)
                except BaseException as exc:
                    tags.append((_name, exc))
                    raise

            setattr(dev, method, wrapped)


def _part_names(path: str, dev: Any) -> List[str]:
    prefix = path.rpartition("/")[0]
    prefix = prefix + "/" if prefix else ""
    return [prefix + (j["name"] if isinstance(j, Mapping) else j.name)
            for j in (dev.fields.get("joints") or [])]


def verify_config(path: Union[str, os.PathLike], timeout: float = 3.0) -> dict:
    """Open the robot of ``path`` read-only, read it, and say which parts sent data.

    Returns ``{"ok", "error", "num_dofs", "joint_pos_len", "parts": [...], "notes"}``.
    Each part is ``{"path", "kind", "status", "note"}`` with status ``seen``,
    ``partial``, ``not_seen`` or ``skipped``. ``ok`` is true when the robot
    connected, ``joint_pos`` has one value per joint and every body joint is
    finite; a hand, camera or microphone without data is reported as not seen
    (please confirm) without failing. Never sends a command; always disconnects.
    """
    from hirobot import load_embodiment
    from hirobot.loading.tree import DeviceSpec, load_robot

    result: Dict[str, Any] = {"ok": False, "error": None, "num_dofs": None,
                              "joint_pos_len": None, "parts": [], "notes": []}
    try:
        emb = load_embodiment(str(path))
        spec = load_robot(str(path))
        skipped = _not_read_only(spec)
    except Exception as exc:  # noqa: BLE001 - reported, not raised
        result["error"] = f"{type(exc).__name__}: {exc}"
        return result
    result["num_dofs"] = emb.num_dofs
    kinds = {p: d.kind for p, d in spec.root.walk() if isinstance(d, DeviceSpec)}
    drop = [s["path"] for s in skipped]
    robot: Any = None
    failed: Dict[str, str] = {}
    connect_error: Optional[str] = None
    # A hand, camera or microphone whose connect raises is left out and the robot
    # opened again without it: at most one retry per part. A body failure ends it.
    for _ in range(len(kinds) + 1):
        try:
            robot = open_read_only(path, drop + list(failed))
        except Exception as exc:  # noqa: BLE001
            result["error"] = f"could not open read-only: {type(exc).__name__}: {exc}"
            result["parts"] = skipped
            return result
        tags: List[Tuple[str, BaseException]] = []
        _tag_connect_failures(robot, tags)
        try:
            robot.connect()
            break
        except Exception as exc:  # noqa: BLE001
            part = tags[-1][0] if tags else None
            if not part or part in failed or kinds.get(part) in (None, "body"):
                connect_error = f"{type(exc).__name__}: {exc}"
                break
            failed[part] = f"{type(exc).__name__}: {exc}"
    else:
        connect_error = "connect kept failing after leaving out " + ", ".join(failed)
    for part, why in failed.items():
        skipped.append({"path": part, "kind": kinds.get(part, "?"), "status": "not_seen",
                        "note": f"connect failed ({why}); left out, please confirm"})
    if connect_error is not None:
        result["error"] = connect_error
        result["parts"] = skipped
        return result
    names = list(robot.embodiment.joint_names)
    obs = None
    mics_seen: Dict[str, bool] = {}
    try:
        deadline = time.monotonic() + max(0.0, float(timeout))
        cameras = list(getattr(robot, "cameras", {}) or {})
        while True:
            obs = robot.get_observation()
            q = np.asarray(obs.joint_pos, dtype=float) if obs.joint_pos is not None else None
            done = (q is not None and q.shape == (len(names),) and bool(np.all(np.isfinite(q)))
                    and all(c in obs.images for c in cameras))
            if done or time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        for name, mic in (getattr(robot, "microphones", {}) or {}).items():
            left = max(0.0, deadline - time.monotonic())
            mics_seen[name] = mic.read(timeout=max(left, 0.2)) is not None
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            robot.disconnect()
        except Exception as exc:  # noqa: BLE001
            result["notes"].append(f"disconnect failed: {type(exc).__name__}: {exc}")
    q = None
    if obs is not None and obs.joint_pos is not None:
        q = np.asarray(obs.joint_pos, dtype=float)
        result["joint_pos_len"] = int(q.size)
    body_ok = False
    skipped_paths = {s["path"] for s in skipped}
    for p, dev in spec.root.walk():
        if not isinstance(dev, DeviceSpec) or p in skipped_paths:
            continue
        entry: Dict[str, Any] = {"path": p, "kind": dev.kind, "status": "not_seen", "note": ""}
        joints = _part_names(p, dev)
        if joints:
            if q is not None and q.size == len(names):
                vals = q[[names.index(j) for j in joints if j in names]]
                n = int(np.isfinite(vals).sum())
                entry["status"] = ("seen" if n == len(joints) else
                                   "partial" if n else "not_seen")
                entry["note"] = f"{n} of {len(joints)} joints report"
                if dev.kind == "body":
                    body_ok = n == len(joints)
        elif dev.kind == "camera":  # a device's name is its part path
            if not dev.driver:
                entry.update(status="skipped", note="no driver")
            elif obs is not None and p in obs.images:
                entry.update(status="seen", note="image "
                             + "x".join(str(s) for s in obs.images[p].shape))
        elif dev.kind == "microphone":
            if not dev.driver:
                entry.update(status="skipped", note="no driver")
            elif mics_seen.get(p):
                entry.update(status="seen", note="audio received")
        else:
            entry.update(status="skipped", note="nothing to read")
        if entry["status"] in ("not_seen", "partial"):
            entry["note"] = (entry["note"] + "; " if entry["note"] else "") + \
                "not seen, please confirm"
        result["parts"].append(entry)
    result["parts"] += skipped
    if q is not None and q.size != len(names):
        result["notes"].append(f"joint_pos has {q.size} values, the robot opened {len(names)}")
    result["ok"] = (result["error"] is None and q is not None and q.size == len(names)
                    and body_ok)
    return result


# ------------------------------------------------------------------ init


def load_answers(answers: Union[None, str, os.PathLike, Mapping[str, Any]]) -> Dict[str, Any]:
    """The ``--answers`` values as a dict; raise ``ValueError`` on an unknown key or bad file."""
    if answers is None:
        return {}
    if isinstance(answers, Mapping):
        data: Any = dict(answers)
    else:
        with open(answers, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    if not isinstance(data, Mapping):
        raise ValueError(f"answers {answers!r}: expected a mapping with {', '.join(ANSWER_KEYS)}")
    unknown = sorted(set(data) - set(ANSWER_KEYS))
    if unknown:
        raise ValueError(f"answers: unknown keys {unknown}; known: {', '.join(ANSWER_KEYS)}")
    for key in ("overwrite", "verify"):
        if key in data and not isinstance(data[key], bool):
            raise ValueError(f"answers: {key} must be true or false, got {data[key]!r}")
    return dict(data)


def _interactive(input_fn: Callable[[str], str]) -> bool:
    """Whether to ask: a caller's own ``input_fn`` always, ``input`` only on a terminal."""
    if input_fn is not builtins.input:
        return True
    try:
        return sys.stdin is not None and sys.stdin.isatty()
    except (AttributeError, ValueError):
        return False


def _candidates(report: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """The scan's model candidates that v1 can write, best first."""
    out: List[Dict[str, Any]] = []
    for cand in ((report or {}).get("unitree") or {}).get("candidates") or []:
        try:
            name = check_model(cand.get("config", ""))
        except ValueError:
            continue
        if name not in [c["config"] for c in out]:
            out.append(dict(cand, config=name))
    return sorted(out, key=lambda c: -float(c.get("confidence") or 0.0))


def _not_seen_message(report: Mapping[str, Any]) -> str:
    uni = dict((report or {}).get("unitree") or {})
    nic = uni.get("interface") or "any network card"
    why = [f"status {uni.get('status') or 'unknown'}"]
    if uni.get("error"):
        why.append(str(uni["error"]))
    why += [str(n) for n in uni.get("notes") or []]
    return (f"no Unitree robot seen on {nic}: {'; '.join(why)}; "
            "pass --model robots/unitree_g1 to write one anyway")


def _pick_model(cands: List[Dict[str, Any]], ask: bool, yes: bool,
                input_fn: Callable[[str], str], print_fn: Callable[..., Any]) -> Optional[str]:
    """The chosen candidate; ``None`` when nobody can answer and the scan is not certain."""
    if yes:
        return cands[0]["config"]
    if not ask:
        best = cands[0]
        return best["config"] if float(best.get("confidence") or 0.0) >= _CERTAIN else None
    print_fn("Models that match what the scan saw (please confirm):")
    for i, c in enumerate(cands, 1):
        why = f"; {c['why']}" if c.get("why") else ""
        print_fn(f"  {i}. {c['config']}  (confidence {float(c.get('confidence') or 0):.2f}{why})")
    while True:
        reply = input_fn(f"Model [1-{len(cands)} or a config name, default 1]: ").strip()
        if not reply:
            return cands[0]["config"]
        if reply.isdigit() and 1 <= int(reply) <= len(cands):
            return cands[int(reply) - 1]["config"]
        try:
            return check_model(reply)
        except ValueError as exc:
            print_fn(f"  {exc}")


def _print_verify(result: Mapping[str, Any], print_fn: Callable[..., Any]) -> None:
    print_fn(f"Read-only check: joint_pos has {result.get('joint_pos_len')} values, "
             f"the embodiment {result.get('num_dofs')} joints")
    for part in result.get("parts") or []:
        status = {"seen": "data seen", "partial": "partly seen", "not_seen": "not seen",
                  "skipped": "skipped"}.get(part["status"], part["status"])
        note = f" ({part['note']})" if part.get("note") else ""
        print_fn(f"  {part['path']:<16} {part['kind']:<12} {status}{note}")
    for note in result.get("notes") or []:
        print_fn(f"  note: {note}")
    if result.get("error"):
        print_fn(f"  error: {result['error']}")


def _may_overwrite(output: Path, answers: Mapping[str, Any], ask: bool, yes: bool, force: bool,
                   input_fn: Callable[[str], str]) -> bool:
    if force or answers.get("overwrite") is True:
        return True
    if "overwrite" in answers or yes or not ask:
        return False
    reply = input_fn(f"{output} exists. Overwrite? [y/N]: ").strip().lower()
    return reply in ("y", "yes")


def run_init(output, *, report=None, model=None, interface=None, answers=None, yes=False,
             force=False, verify=True, timeout=2.0, input_fn=input, print_fn=print) -> int:
    """Scan, pick the model and NIC, write the local config to ``output`` and check it read-only.

    Returns 0 on success, 1 when nothing was written (no model, no NIC, the file
    exists and may not be overwritten, a bad answer) and 2 when the file was
    written but the read-only check failed (the file is kept).
    """
    from hirobot import load_embodiment

    def err(msg: str) -> int:
        print_fn(f"hirobot init: {msg}")
        return 1

    try:
        answers = load_answers(answers)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return err(str(exc))
    ask = _interactive(input_fn)
    interface = interface or answers.get("interface")
    if report is None:
        scan_mod = importlib.import_module("hirobot.scan")
        report = scan_mod.scan(interface, timeout)
    report = dict(report or {})

    # 1. the model
    chosen = model or answers.get("model")
    if chosen:
        try:
            chosen = check_model(chosen)
        except ValueError as exc:
            return err(str(exc))
    else:
        cands = _candidates(report)
        if not cands:
            return err(_not_seen_message(report))
        chosen = _pick_model(cands, ask, yes, input_fn, print_fn)
        if chosen is None:
            listed = ", ".join(f"{c['config']} ({float(c.get('confidence') or 0):.2f})"
                               for c in cands)
            return err(f"the scan is not certain which robot this is ({listed}); "
                       "pass --model, --yes (takes the first) or an answers file")

    # 2. the network card
    nic = interface or ((report.get("unitree") or {}).get("interface"))
    if not nic:
        return err("no network card on the robot's network was found; pass --interface "
                   "(see the suggestions of `hirobot scan`)")

    # 3. the file
    out = Path(output)
    if out.exists() and not _may_overwrite(out, answers, ask, yes, force, input_fn):
        return err(f"{out} exists and was kept; pass --force or answer overwrite: true "
                   "to replace it")
    try:
        text = render_config(chosen, nic, report)
    except ValueError as exc:
        return err(str(exc))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    try:
        emb = load_embodiment(str(out))
    except Exception as exc:  # noqa: BLE001 - reported with the path
        return err(f"wrote {out}, but it does not load: {type(exc).__name__}: {exc}")
    print_fn(f"Wrote {out}: {chosen} on {nic}, {emb.num_dofs} joints.")

    # 4. the read-only check
    if not verify or answers.get("verify") is False:
        print_fn("Read-only check skipped. Check the file before the first commanded motion "
                 "(docs/setup/unitree.md).")
        return 0
    result = verify_config(out, timeout=max(float(timeout), 1.0))
    _print_verify(result, print_fn)
    if not result["ok"]:
        print_fn(f"hirobot init: the read-only check failed; {out} is kept. Fix the cause "
                 "(robot on, network card) and run `hirobot init` again, or edit the file.")
        return 2
    print_fn("Read-only check passed. Check the file before the first commanded motion "
             "(docs/setup/unitree.md).")
    return 0


__all__ = ["check_model", "load_answers", "part_overrides", "render_config", "run_init",
           "unitree_models", "verify_config"]
