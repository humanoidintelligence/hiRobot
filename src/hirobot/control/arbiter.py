"""Who may command a robot: one holder at a time, by priority.

A :class:`CommandArbiter` sits on the outermost robot of a stack
(:func:`~hirobot.make_robot` attaches one). A caller takes the command stream
with :meth:`Robot.claim <hirobot.core.robot.Robot.claim>` and sends through the
returned :class:`CommandLease`::

    with robot.claim("runner") as r:
        r.send_action(action)

While a lease is held, calling ``send_action``, ``hold``, ``reset`` or
``run_robot_skill`` on the robot directly raises :class:`RobotBusyError`.
``estop``, ``get_observation``, ``get_status`` and ``now`` are never checked.
With nobody holding, everyone may command (single-caller scripts keep working)
unless the arbiter was built with ``require_claim=True``.

A higher :class:`Priority` takes the stream at once: the old lease is marked
preempted (its next call raises :class:`PreemptedError`), its ``on_preempt``
callback runs, and the robot is told to :meth:`hold` so it does not keep
going toward the old target. An equal or lower priority raises
:class:`RobotBusyError`. A released stream is not given back to a preempted
holder; it has to claim again.

Commands a layer sends to the robot below it do not go through the arbiter:
the arbiter sits above every layer.

A claim may carry an ``envelope``: limits stricter than the robot's gate
(``joint_limit``, ``max_joint_step``, ``max_joint_error``, ``gripper_range``,
``allowed_modes``) that apply to that lease's commands only, on top of the
outermost gate. They do not reach the gate under the layers, so a whole-body
controller's own joint commands are never limited by a caller's envelope.

**Leases on parts.** On a robot assembled from parts (an ``assembly:``
config), ``claim(name, parts=["left"])`` holds only those parts; a path
covers its whole subtree (``"cell"`` covers ``"cell/left"``), and no
``parts`` (or ``""``) is the whole tree. Leases whose parts do not overlap
are held at the same time, each sending only its own parts' commands (an
action that touches a part outside the lease raises :class:`RobotBusyError`;
the parts it leaves out keep their targets). Overlapping claims follow the
priority rule above; a preempted lease loses all its parts, and only its
parts are held. A lease's ``envelope`` limits only its parts' joints. With
part leases held, a direct call that touches no held part still goes
through (``hold`` and ``reset`` touch the whole tree). A robot without
parts has one part, the whole robot, so all of this is the rule above.

The arbiter keeps an audit: a ring of events (claims, releases, preemptions,
refused commands, busy refusals; :meth:`CommandArbiter.events`, each with the
lease's parts) and per-lease counts of accepted and refused commands.
``get_status().safety["audit"]`` has the counts, the holders (part path ->
lease name, ``""`` for a whole-tree lease) and the last refusal.
"""

from __future__ import annotations

import enum
import logging
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import (TYPE_CHECKING, Any, Callable, Deque, Dict, Iterator, List, Mapping, Optional,
                    Sequence, Tuple)

if TYPE_CHECKING:  # pragma: no cover
    from hirobot.core.robot import Robot

logger = logging.getLogger(__name__)

#: Seconds a claim waits for the old holder's command in flight before it
#: holds the robot anyway (the old holder may be in a long ``reset``).
PREEMPT_LOCK_TIMEOUT_S = 0.2

#: Events the audit ring keeps (``arbiter.audit_size`` in the config).
AUDIT_SIZE = 256

#: Lease names the per-lease counts keep; the oldest is dropped past this.
MAX_LEASE_COUNTS = 64


class Priority(enum.IntEnum):
    """Who may preempt whom: a higher value takes the stream from a lower one."""

    CALLER = 0  #: An application: hiAgent's runner, a benchmark loop, a script.
    LAYER_SKILL = 1  #: A layer while its layer skill runs.
    TAKEOVER = 2  #: A person taking over: teleoperation, a joystick.


class RobotBusyError(RuntimeError):
    """Another lease holds the robot, or the robot needs a claim first."""


class PreemptedError(RobotBusyError):
    """This lease lost the robot to a higher priority; claim again to continue."""


@dataclass
class CommandEvent:
    """One audit entry.

    Attributes:
        t: The robot's clock (:meth:`~hirobot.core.robot.Robot.now`).
        wall: ``time.time()``.
        kind: ``claim``, ``release``, ``preempted``, ``refused`` (the gate or the
            envelope refused a command), ``busy`` (a call refused because
            another lease holds the robot, or after a preemption / release).
        lease: Lease name, or ``None`` for a direct call with nobody holding.
        priority: The lease's priority name, or ``None``.
        call: The method (``send_action``, ``hold``, ...) for ``refused`` / ``busy``.
        detail: The refusal reason, or who preempted whom.
        parts: The lease's part paths, or ``None`` for the whole tree (and for
            a direct call).
    """

    t: float
    wall: float
    kind: str
    lease: Optional[str]
    priority: Optional[str]
    call: Optional[str] = None
    detail: str = ""
    parts: Optional[List[str]] = None


@dataclass
class _Counts:
    accepted: int = 0
    refused: int = 0
    by_lease: Dict[str, Dict[str, int]] = field(default_factory=dict)


class CommandLease:
    """The handle a holder commands the robot through.

    Methods mirror the robot's state-changing calls and raise
    :class:`PreemptedError` once preempted and :class:`RobotBusyError` once
    released. Reads go to :attr:`robot` directly. A context manager: leaving
    the ``with`` block releases the lease.
    """

    def __init__(self, arbiter: "CommandArbiter", name: str, priority: Priority,
                 on_preempt: Optional[Callable[["CommandLease"], None]],
                 envelope_gate: Any = None, envelope: Optional[Dict[str, Any]] = None,
                 parts: Optional[List[str]] = None) -> None:
        self.arbiter = arbiter
        self.name = name
        self.priority = Priority(priority)
        self.on_preempt = on_preempt
        #: Part paths this lease holds, or ``None`` for the whole tree.
        self.parts: Optional[List[str]] = None if parts is None else list(parts)
        #: The envelope items in force (stricter than the robot's gate), or ``{}``.
        self.envelope: Dict[str, Any] = dict(envelope or {})
        self._envelope_gate = envelope_gate
        #: Commands this lease sent that were accepted / refused.
        self.accepted = 0
        self.refused = 0
        #: Name of the lease that took the robot, once preempted.
        self.preempted_by: Optional[str] = None
        self.released = False
        # Held by this lease's call in flight, so a preemption's hold() comes after it.
        self._lock = threading.RLock()
        #: Resolved parts (:func:`hirobot.assembly.parts.resolve_part`), ``None`` for the whole robot.
        self._scope: Optional[List[Tuple[str, bool]]] = None

    @property
    def robot(self) -> "Robot":
        return self.arbiter.robot

    @property
    def active(self) -> bool:
        return self.arbiter._is_active(self)

    @property
    def preempted(self) -> bool:
        return self.preempted_by is not None

    # Every call runs the robot's own method inside the arbiter's admission,
    # so nested calls (``hold`` sending a POSITION action) pass too.

    def send_action(self, action: Any) -> None:
        """Send through the lease: the envelope first (if any), then the robot's own checks.

        A lease on some parts raises :class:`RobotBusyError` for an action that
        touches a part outside them; nothing is sent.
        """
        arbiter = self.arbiter
        with arbiter._admit(self, "send_action", touched=lambda: _touched(
                self.robot, "send_action", (action,))):
            with arbiter._send_lock:  # check, send and commit as one, between leases
                if self._envelope_gate is not None:
                    action = self._check_envelope(action)
                self.robot.send_action(action)
                if self._envelope_gate is not None:
                    self._envelope_gate.commit(action)

    def _check_envelope(self, action: Any) -> Any:
        from hirobot.assembly.parts import action_leaves
        from hirobot.control.safety import SafetyError

        gate = self._envelope_gate
        modes = ([sub.mode for _, sub in action_leaves(action)] if action.parts
                 else [action.mode])
        bad = [m for m in modes if gate.allowed_modes is not None and m not in gate.allowed_modes]
        if bad:
            raise SafetyError(f"envelope of lease {self.name!r}: mode {bad[0].value!r} not "
                              f"allowed; allowed_modes: {[m.value for m in gate.allowed_modes]}")
        verdict = gate.check(action, self.robot.embodiment)
        if not verdict.allowed:
            raise SafetyError(f"envelope of lease {self.name!r}: {verdict.reason}")
        try:
            return gate.check_state(verdict.action, self.robot)
        except SafetyError as exc:
            raise SafetyError(f"envelope of lease {self.name!r}: {exc}") from None

    def hold(self) -> None:
        """Hold the robot; a lease on some parts holds only those parts."""
        with self.arbiter._admit(self, "hold", touched=self._own_nodes):
            self.arbiter._hold_parts(self.parts)

    def reset(self, seed: Optional[int] = None) -> None:
        """Reset the robot; a lease on some parts resets only those parts.

        A part reset leaves the robot's own fault state alone: after an
        e-stop or a part failure only a whole-tree reset clears it.
        """
        with self.arbiter._admit(self, "reset", touched=self._own_nodes):
            if self.parts is None:
                self.robot.reset(seed=seed)
            else:
                self.arbiter._reset_parts(self.parts, seed)

    def run_robot_skill(self, name: str, args: Optional[Mapping[str, Any]] = None) -> Any:
        """Run a robot skill; a lease on some parts only those parts' skills (``<part>/<skill>``)."""
        with self.arbiter._admit(self, "run_robot_skill", touched=lambda: _touched(
                self.robot, "run_robot_skill", (name,))):
            return self.robot.run_robot_skill(name, args)

    def _own_nodes(self) -> Optional[set]:
        """The lease's own parts as touched nodes (its hold / reset stay inside them)."""
        return None if self._scope is None else {node for node, _ in self._scope}

    @contextmanager
    def _admitted(self, what: str) -> Iterator[None]:
        """Run a call the holder makes some other way (a group sending a member
        an action it already checked) as if it came through this lease.

        The call counts as commanding the lease's own parts (a layer skill on
        one part starts under a lease on that part)."""
        scope = self._scope
        touched = None if scope is None else (lambda: {node for node, _ in scope})
        with self.arbiter._admit(self, what, touched=touched):
            yield

    def release(self) -> None:
        """Give the robot back. Safe to call twice, and after a preemption."""
        self.arbiter._release(self)

    def __enter__(self) -> "CommandLease":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.release()

    def __repr__(self) -> str:
        state = ("preempted by " + self.preempted_by if self.preempted
                 else "released" if self.released else "active")
        return f"CommandLease({self.name!r}, {self.priority.name}, {state})"


class CommandArbiter:
    """The command stream of one robot, held by leases on parts of it (see the module docstring).

    Args:
        robot: The robot whose calls are checked (the outermost of the stack).
        require_claim: Refuse direct commands: every command goes through a lease.
    """

    def __init__(self, robot: "Robot", require_claim: bool = False,
                 audit_size: int = AUDIT_SIZE) -> None:
        self.robot = robot
        self.require_claim = bool(require_claim)
        if int(audit_size) < 1:
            raise ValueError(f"arbiter.audit_size must be >= 1, got {audit_size}")
        self._events: Deque[CommandEvent] = deque(maxlen=int(audit_size))
        self._counts = _Counts()
        self._last_refusal: Optional[CommandEvent] = None
        self._audit = threading.Lock()
        #: Active leases, oldest first.
        self._leases: List[CommandLease] = []
        self._state = threading.Lock()
        # Leases on different parts send from different threads: one at a time,
        # so the gate's step reference sees each command whole.
        self._send_lock = threading.RLock()
        self._local = threading.local()

    @property
    def holder(self) -> Optional[CommandLease]:
        """The lease on the whole robot, else the oldest part lease, else ``None``."""
        leases = list(self._leases)
        whole = [lease for lease in leases if lease.parts is None]
        return (whole or leases or [None])[0]

    @property
    def holders(self) -> List[CommandLease]:
        """Every active lease, oldest first."""
        return list(self._leases)

    def claim(self, name: str, priority: Priority = Priority.CALLER,
              on_preempt: Optional[Callable[[CommandLease], None]] = None,
              envelope: Optional[Mapping[str, Any]] = None,
              parts: Optional[Sequence[str]] = None) -> CommandLease:
        """Take the command stream of ``parts`` (default: the whole robot).

        Raises :class:`RobotBusyError` when a lease on an overlapping part has
        an equal or higher priority, and ``ValueError`` for an unknown part.
        Overlapping leases of lower priority are preempted: each one's
        ``on_preempt`` runs (errors are logged), then its parts hold; a hold
        that fails (the robot is in the fault state) is logged and the claim
        still succeeds.

        ``envelope`` sets limits for this lease's ``send_action`` (see the
        module docstring), on the lease's joints only; items not stricter than
        the robot's gate are ignored with a warning, and a bad key or value
        raises ``ValueError`` before anything is taken. Fixed for the lease's
        life: release and claim again to change it.
        """
        scope, paths = self._scope(parts)
        gate = applied = None
        if envelope:
            from hirobot.control.safety import envelope_gate

            gate, applied = envelope_gate(getattr(self.robot, "safety", None), envelope,
                                          self.robot, name, joints=self._joints(paths))
        lease = CommandLease(self, name, Priority(priority), on_preempt, gate, applied,
                             parts=paths)
        lease._scope = scope
        with self._state:
            hit = [old for old in self._leases if _overlap(old._scope, scope)]
            strong = [old for old in hit if old.priority >= lease.priority]
            if strong:
                old = strong[0]
                err = RobotBusyError(
                    f"{type(self.robot).__name__}{_where(old.parts)} is held by {old.name!r} "
                    f"({old.priority.name}); {name!r} ({lease.priority.name}) cannot take it")
                self._record("busy", lease, "claim", str(err))
                raise err
            for old in hit:
                old.preempted_by = name
                self._leases.remove(old)
            self._leases.append(lease)
        self._record("claim", lease, detail=(f"envelope {applied}" if applied else ""))
        for old in hit:
            self._record("preempted", old, detail=f"by {name!r} ({lease.priority.name})")
            logger.warning("%s: %r (%s) preempted %r (%s)", type(self.robot).__name__,
                           name, lease.priority.name, old.name, old.priority.name)
            if old.on_preempt is not None:
                try:
                    old.on_preempt(old)
                except Exception:  # noqa: BLE001 - the new holder still gets the robot
                    logger.exception("on_preempt of %r failed", old.name)
            self._hold_for(lease, old)
        return lease

    def _scope(self, parts: Optional[Sequence[str]]) -> Tuple[Optional[List[Tuple[str, bool]]],
                                                               Optional[List[str]]]:
        """``(resolved, paths)``: :func:`hirobot.assembly.parts.resolve_part` of each part path, and
        the paths as given; ``(None, None)`` for the whole robot."""
        if parts is None:
            return None, None
        if isinstance(parts, str):
            parts = [parts]
        from hirobot.assembly.parts import resolve_part

        emb = self.robot.embodiment
        if not parts:
            raise ValueError("claim(parts=[]): name at least one part, or leave parts out")
        scope, names = [], []
        for p in parts:
            if not emb.parts and str(p).strip("/"):
                raise ValueError(f"claim(parts={list(parts)}): this robot has no parts")
            node, own_only = resolve_part(emb, str(p))
            if not node and not own_only:
                return None, None  # "" is the whole robot
            scope.append((node, own_only))
            names.append(str(p).strip("/"))
        return scope, names

    def _joints(self, paths: Optional[List[str]]) -> Optional[List[int]]:
        """Joint indices the part paths cover, or ``None`` for the whole robot."""
        if paths is None:
            return None
        from hirobot.assembly.parts import scope_joints

        return scope_joints(self.robot.embodiment, paths)

    def _is_active(self, lease: CommandLease) -> bool:
        return lease in self._leases

    def _hold_for(self, lease: CommandLease, old: CommandLease) -> None:
        """Hold the preempted lease's parts once its call in flight is done."""
        got = old._lock.acquire(timeout=PREEMPT_LOCK_TIMEOUT_S)
        if not got:
            logger.warning("%s: the preempted holder's call did not finish within %.2f s; "
                           "holding anyway", type(self.robot).__name__, PREEMPT_LOCK_TIMEOUT_S)
        try:
            with self._admit(lease, "hold", lock=False):
                self._hold_parts(old.parts)
        except Exception as exc:  # noqa: BLE001 - e.g. the robot is in the fault state
            logger.warning("%s: hold() on preemption failed: %r", type(self.robot).__name__, exc)
        finally:
            if got:
                old._lock.release()

    def _hold_parts(self, parts: Optional[List[str]]) -> None:
        """Hold ``parts`` (``None``: the whole robot)."""
        hold_parts = getattr(self.robot, "hold_parts", None)
        if parts is None or hold_parts is None:
            # A layer on top drives the whole robot: holding it holds everything.
            self.robot.hold()
        else:
            hold_parts(parts)

    def _reset_parts(self, parts: List[str], seed: Optional[int]) -> None:
        reset_parts = getattr(self.robot, "reset_parts", None)
        if reset_parts is None:
            raise RobotBusyError(f"{type(self.robot).__name__} resets only as a whole; claim "
                                 "the whole robot to reset it")
        reset_parts(parts, seed)

    def _release(self, lease: CommandLease) -> None:
        with self._state:
            first = not lease.released
            lease.released = True
            held = lease in self._leases
            if held:
                self._leases.remove(lease)
        if first and held:
            self._record("release", lease)

    # ---------------------------------------------------------------- audit

    def events(self) -> List[CommandEvent]:
        """The audit ring, oldest first (a copy)."""
        with self._audit:
            return list(self._events)

    def audit(self) -> Dict[str, Any]:
        """Counts for ``get_status().safety["audit"]``: totals, per lease, the last refusal."""
        with self._audit:
            last = self._last_refusal
            return {
                "accepted": self._counts.accepted,
                "refused": self._counts.refused,
                "by_lease": {k: dict(v) for k, v in self._counts.by_lease.items()},
                "holder": None if self.holder is None else self.holder.name,
                "holders": {("" if lease.parts is None else ",".join(lease.parts)): lease.name
                            for lease in self._leases},
                "last_refusal": None if last is None else {
                    "t": last.t, "lease": last.lease, "call": last.call, "reason": last.detail},
            }

    def _record(self, kind: str, lease: Optional[CommandLease], call: Optional[str] = None,
                detail: str = "") -> None:
        try:
            t = float(self.robot.now())
        except Exception:  # noqa: BLE001 - the audit must not break a call
            t = time.monotonic()
        event = CommandEvent(t=t, wall=time.time(), kind=kind,
                             lease=None if lease is None else lease.name,
                             priority=None if lease is None else lease.priority.name,
                             call=call, detail=detail,
                             parts=None if lease is None or lease.parts is None
                             else list(lease.parts))
        with self._audit:
            self._events.append(event)
            if kind == "refused":
                self._last_refusal = event
        if kind in ("refused", "busy", "preempted"):
            logger.info("%s audit: %s %s %s %s", type(self.robot).__name__, kind, event.lease,
                        call or "", detail)

    def _count(self, lease: Optional[CommandLease], accepted: bool) -> None:
        key = "(direct)" if lease is None else lease.name
        with self._audit:
            by_lease = self._counts.by_lease
            if key not in by_lease and len(by_lease) >= MAX_LEASE_COUNTS:
                del by_lease[next(iter(by_lease))]  # the oldest name
            counts = by_lease.setdefault(key, {"accepted": 0, "refused": 0})
            if accepted:
                self._counts.accepted += 1
                counts["accepted"] += 1
            else:
                self._counts.refused += 1
                counts["refused"] += 1
        if lease is not None:
            if accepted:
                lease.accepted += 1
            else:
                lease.refused += 1

    # ------------------------------------------------------------ admission

    @contextmanager
    def _admit(self, lease: Optional[CommandLease], what: str, lock: bool = True,
               touched: Optional[Callable[[], Optional[set]]] = None) -> Iterator[None]:
        """Let a call through, or raise. ``lease`` is ``None`` for a direct call.

        ``touched`` gives the nodes the call commands (``None``: the whole
        robot); a lease on some parts may only touch those, and a direct call
        may not touch a held part. A top-level call is audited: a refusal by
        the gate or the envelope (``SafetyError``) is recorded, and
        ``send_action`` is counted.
        """
        local = self._local
        if getattr(local, "depth", 0):
            # Nested in an admitted call on this thread (hold() -> send_action()).
            current = local.lease
            if current is not None and current not in self._leases:
                raise self._lost(current, what)
            yield
            return
        nodes = self._touched_nodes(touched, lease, what)
        if lease is None:
            busy = [h for h in self._leases if nodes is None or h._scope is None
                    or any(_covers(h._scope, n) for n in nodes)]
            if busy:
                h = busy[0]
                err = RobotBusyError(
                    f"{type(self.robot).__name__}{_where(h.parts)} is held by {h.name!r} "
                    f"({h.priority.name}); {what} goes through its lease")
                self._record("busy", None, what, str(err))
                raise err
            if self.require_claim:
                err = RobotBusyError(
                    f"{type(self.robot).__name__} needs a claim first (arbiter.require_claim); "
                    f"use robot.claim(name) and call {what} on the lease")
                self._record("busy", None, what, str(err))
                raise err
            # A direct call: not serialized against leases, but audited.
            local.depth, local.lease = 1, None
            try:
                with self._audited(None, what):
                    yield
            finally:
                local.depth = 0
            return
        if lease not in self._leases:
            err = self._lost(lease, what)
            self._record("busy", lease, what, str(err))
            raise err
        if lease._scope is not None:
            outside = sorted(n or "(own body)" for n in (nodes or {None})
                             if n is None or not _covers(lease._scope, n))
            if outside:
                err = RobotBusyError(
                    f"lease {lease.name!r} holds {lease.parts}; {what} also commands "
                    f"{['the whole robot'] if nodes is None else outside}")
                self._record("busy", lease, what, str(err))
                raise err
        got = False
        if lock:
            got = lease._lock.acquire(timeout=PREEMPT_LOCK_TIMEOUT_S)
            if not got:
                logger.warning("%s: %s waited %.2f s for a call in flight; going ahead",
                               type(self.robot).__name__, what, PREEMPT_LOCK_TIMEOUT_S)
        local.depth, local.lease = 1, lease
        try:
            with self._audited(lease, what):
                yield
        finally:
            local.depth, local.lease = 0, None
            if got:
                lease._lock.release()

    def _touched_nodes(self, touched: Optional[Callable[[], Optional[set]]],
                       lease: Optional[CommandLease], what: str) -> Optional[set]:
        """The nodes a call commands; ``None`` (the whole robot) without ``touched``."""
        if touched is None or not self.robot.embodiment.parts:
            return None
        if not self._leases:
            return None  # nobody holds a part: no need to work it out
        from hirobot.control.safety import SafetyError

        try:
            return touched()
        except SafetyError as exc:  # a malformed action: the gate refuses it with the reason
            logger.debug("%s: %s: %s", type(self.robot).__name__, what, exc)
            return None

    @contextmanager
    def _audited(self, lease: Optional[CommandLease], what: str) -> Iterator[None]:
        from hirobot.control.safety import SafetyError

        try:
            yield
        except SafetyError as exc:
            self._record("refused", lease, what, str(exc))
            if what == "send_action":
                self._count(lease, accepted=False)
            raise
        if what == "send_action":
            self._count(lease, accepted=True)

    def _lost(self, lease: CommandLease, what: str) -> RobotBusyError:
        if lease.preempted:
            return PreemptedError(f"lease {lease.name!r} was preempted by {lease.preempted_by!r}; "
                                  f"{what} refused (claim again to continue)")
        return RobotBusyError(f"lease {lease.name!r} was released; {what} refused")


def _touched(robot: "Robot", what: str, args: Sequence[Any]) -> Optional[set]:
    """The nodes a call commands (``embodiment.parts`` keys), ``None`` for the whole robot.

    ``send_action``: what the action names (:func:`hirobot.assembly.parts.action_owners`).
    ``run_robot_skill``: the node of the skill's ``<part>/`` prefix. ``hold``
    and ``reset`` on the robot are the whole robot.
    """
    from hirobot.assembly.parts import action_owners, owner_part

    emb = robot.embodiment
    if what == "send_action" and args:
        return action_owners(emb, args[0])
    if what == "run_robot_skill" and args:
        return {owner_part(emb, str(args[0]))}
    return None


def _covers(scope: Optional[List[Tuple[str, bool]]], node: str) -> bool:
    from hirobot.assembly.parts import part_covers

    return scope is None or any(part_covers(p, node) for p in scope)


def _overlap(a: Optional[List[Tuple[str, bool]]], b: Optional[List[Tuple[str, bool]]]) -> bool:
    from hirobot.assembly.parts import parts_overlap

    return a is None or b is None or any(parts_overlap(x, y) for x in a for y in b)


def _where(parts: Optional[List[str]]) -> str:
    return "" if parts is None else f" (parts {parts})"


__all__ = ["AUDIT_SIZE", "CommandArbiter", "CommandEvent", "CommandLease",
           "PREEMPT_LOCK_TIMEOUT_S", "PreemptedError", "Priority", "RobotBusyError"]
