"""The Unitree G1's built-in skills, exposed as hiRobot onboard skills.

The list is fixed here, taken from unitree_sdk2_python (a035ade, 2025-11-28);
hiRobot does not ask the robot for it. Two RPC services of the G1's built-in
motion service (``ai``) carry the skills out:

* ``sport`` (``unitree_sdk2py.g1.loco.LocoClient``): the locomotion state machine
  (FSM ids as in ``LocoClient.Start`` / ``Sit`` / ...), standing height and
  velocity commands.
* ``arm`` (``unitree_sdk2py.g1.arm.G1ArmActionClient``): canned arm gestures,
  ids from the SDK's ``action_map``. ``release arm`` (99) is not a skill:
  :meth:`G1Skills.stop` sends it.

Left out on purpose: ``ZeroTorque`` (FSM 0, the robot drops with no damping),
``SetBalanceMode`` and the ``LocoClient`` wave / shake-hand task ids, which
repeat the arm gestures.

Both services answer only while the built-in service is active. When it is not
(released by ``release_motion_mode``, or not yet selected), running a skill
fails with RPC code 3102 and a clear error; the list itself does not change.

``ExecuteAction`` replies only when the gesture has finished (``high_wave`` takes
more than 3 s), so a gesture runs on a background thread with a long RPC timeout.
:meth:`G1Skills.run` waits ``accept_s`` for an early reply: an error code in that
time (wrong FSM, arm busy) raises, otherwise the gesture counts as accepted and
its final code is only logged.

``turn`` is closed-loop: it reads the IMU yaw and keeps sending short
``SetVelocity`` commands until the robot faces the target, then ``StopMove``. It
runs on a background thread like a gesture; :meth:`G1Skills.stop` or another skill
ends it. Open-loop ``walk(vyaw=0.5, seconds=1.05)`` turned about 22 degrees on the
real G1 instead of 30, which is why ``turn`` does not time its commands from the
rate. Its ``duration_s`` (:meth:`G1Skills.turn_estimate_s`) is only an upper estimate.

Run on a real G1 (2026-11-13 / 16): FSM 0 -> 1 -> 4 -> 500, ``walk``,
``right_hand_up`` and ``high_wave``; the other skills (``turn`` too) have not been tried.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from hirobot.core.robot_skills import RobotSkillError, RobotSkillSpec

logger = logging.getLogger(__name__)

#: unitree_sdk2py.rpc.internal.RPC_ERR_CLIENT_SEND: no service answered.
_RPC_ERR_CLIENT_SEND = 3102
#: Arm action that hands the arms back to the locomotion controller.
_RELEASE_ARM = 99
#: LocoClient.HighStand / LowStand send these as the standing height.
_UINT32_MAX = (1 << 32) - 1

_ON_GROUND = "robot standing on the ground with space around it"
_LOCO = "built-in locomotion running (start_locomotion first)"

#: Conservative walking limits; the controller accepts more.
WALK_LIMITS = {"vx": 0.5, "vy": 0.3, "vyaw": 0.8, "seconds": 10.0}
#: ``turn``: largest angle per call (degrees), fastest / slowest turn rate (rad/s),
#: how close counts as there (degrees), and how long a single ``SetVelocity`` lasts
#: (s), so the robot stops on its own if the loop dies.
TURN_LIMITS = {"degrees": 180.0, "vyaw_max": 0.6, "vyaw_min": 0.15, "tolerance": 2.0,
               "command_s": 0.4}


def _num(desc: str, limit: float) -> Dict[str, Any]:
    return {"type": "number", "minimum": -limit, "maximum": limit, "default": 0.0,
            "description": desc}


def _spec(name: str, description: str, requires: List[str], **kw: Any) -> RobotSkillSpec:
    return RobotSkillSpec(name=name, description=description, requires=requires, **kw)


#: Locomotion skills: name -> (spec, how it is sent). The second item is
#: ("fsm", id), ("height", value), ("walk", None) or ("turn", None).
_LOCO_SKILLS: Dict[str, Tuple[RobotSkillSpec, Tuple[str, Optional[int]]]] = {
    "start_locomotion": (_spec(
        "start_locomotion",
        "Enter the built-in balance controller (FSM 200) so the robot stands and can "
        "walk on its own.", [_ON_GROUND]), ("fsm", 200)),
    "walk": (_spec(
        "walk", "Walk with a body-frame velocity for a number of seconds, then stop.",
        [_ON_GROUND, _LOCO],
        parameters={
            "type": "object",
            "properties": {
                "vx": _num("Forward speed, m/s (negative walks backwards).", WALK_LIMITS["vx"]),
                "vy": _num("Sideways speed, m/s (positive is to the left).", WALK_LIMITS["vy"]),
                "vyaw": _num("Turn rate, rad/s (positive turns left).", WALK_LIMITS["vyaw"]),
                "seconds": {"type": "number", "exclusiveMinimum": 0,
                            "maximum": WALK_LIMITS["seconds"], "default": 1.0,
                            "description": "How long to walk."},
            },
        }), ("walk", None)),
    "turn": (_spec(
        "turn", "Turn on the spot by an angle, measured with the IMU, then stop. "
        "Positive degrees turn left (counter-clockwise seen from above), negative "
        "turn right.",
        [_ON_GROUND, _LOCO],
        parameters={
            "type": "object",
            "properties": {
                "degrees": {**_num("Angle to turn, degrees (positive is left).",
                                   TURN_LIMITS["degrees"]), "default": 30.0},
            },
        }), ("turn", None)),
    "high_stand": (_spec(
        "high_stand", "Stand at the highest standing height.", [_LOCO]),
        ("height", _UINT32_MAX)),
    "low_stand": (_spec(
        "low_stand", "Stand at the lowest standing height.", [_LOCO]), ("height", 0)),
    "sit": (_spec("sit", "Sit down (FSM 3).", [_ON_GROUND, _LOCO]), ("fsm", 3)),
    "squat_stand": (_spec(
        "squat_stand", "Squat down, or stand up from a squat (FSM 706).",
        [_ON_GROUND, _LOCO]), ("fsm", 706)),
    "lie_stand_up": (_spec(
        "lie_stand_up", "Stand up from lying on the ground (FSM 702).",
        ["robot lying on the ground with space around it"]), ("fsm", 702)),
    "damp": (_spec(
        "damp", "Go limp with damping (FSM 1). The robot falls if nothing holds it.",
        ["robot hung or supported"]), ("fsm", 1)),
}

#: Arm gestures: skill name -> (SDK action_map id, description).
GESTURES: Dict[str, Tuple[int, str]] = {
    "shake_hand": (27, "Hold out the right hand for a handshake."),
    "high_five": (18, "Raise a hand for a high five."),
    "hug": (19, "Open both arms for a hug."),
    "high_wave": (26, "Wave a hand above the head."),
    "face_wave": (25, "Wave a hand in front of the face."),
    "clap": (17, "Clap the hands."),
    "hands_up": (15, "Raise both hands."),
    "right_hand_up": (23, "Raise the right hand."),
    "heart": (20, "Make a heart shape with both hands."),
    "right_heart": (21, "Make a heart shape with the right hand."),
    "two_hand_kiss": (11, "Blow a kiss with both hands."),
    "left_kiss": (12, "Blow a kiss with the left hand."),
    "right_kiss": (13, "Blow a kiss with the right hand."),
    "reject": (22, "Make a refusing gesture."),
    "x_ray": (24, "Cross the arms in an X."),
}
_GESTURE_REQUIRES = [_LOCO, "nothing within arm's reach"]

#: Every G1 skill, in the order shown to callers.
SKILLS: List[RobotSkillSpec] = [spec for spec, _ in _LOCO_SKILLS.values()] + [
    _spec(name, f"Arm gesture: {desc}", list(_GESTURE_REQUIRES))
    for name, (_, desc) in GESTURES.items()
]


class G1Skills:
    """Runs the G1 skills over the ``sport`` and ``arm`` RPC services.

    Args:
        loco_client / arm_client: Factories for the SDK clients, injectable for
            tests. Called on first use, after DDS is initialised.
        timeout: RPC timeout of the ``sport`` calls, seconds.
        gesture_timeout: RPC timeout of an arm gesture, seconds; the reply
            comes when the gesture ends.
        accept_s: How long :meth:`run` waits for an early error before it
            returns with the gesture still running.
        yaw: Returns the body yaw from the IMU in radians, or ``None`` when the
            reading is stale. ``turn`` needs it; without it ``turn`` is refused.
        turn_period: Seconds between two steps of the ``turn`` loop.
        turn_timeout: ``turn`` gives up (stops and logs) after this many seconds.
    """

    def __init__(
        self,
        loco_client: Optional[Callable[[], Any]] = None,
        arm_client: Optional[Callable[[], Any]] = None,
        timeout: float = 3.0,
        gesture_timeout: float = 15.0,
        accept_s: float = 0.5,
        yaw: Optional[Callable[[], Optional[float]]] = None,
        turn_period: float = 0.05,
        turn_timeout: float = 15.0,
    ) -> None:
        self._make_loco = loco_client or _sdk_loco_client
        self._make_arm = arm_client or _sdk_arm_client
        self._timeout = timeout
        self._gesture_timeout = gesture_timeout
        self._accept_s = accept_s
        self._yaw = yaw
        self._turn_period = turn_period
        self._turn_timeout = turn_timeout
        self._loco = None
        self._arm = None
        self._turn_cancel: Optional[threading.Event] = None
        self._turn_thread: Optional[threading.Thread] = None

    def _loco_client(self):
        if self._loco is None:
            self._loco = self._make_loco()
            self._loco.SetTimeout(self._timeout)
            self._loco.Init()
        return self._loco

    def _arm_client(self):
        if self._arm is None:
            self._arm = self._make_arm()
            self._arm.SetTimeout(self._gesture_timeout)
            self._arm.Init()
        return self._arm

    def run(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> Optional[float]:
        """Start ``spec`` with checked ``args``; returns once the robot accepts it.

        Returns the expected run time for these arguments (``walk``: its
        ``seconds``; ``turn``: :meth:`turn_estimate_s`), or ``None`` for the
        spec's own ``duration_s``.
        """
        if spec.name in GESTURES:
            _check(spec.name, self._arm_action(spec.name, GESTURES[spec.name][0]))
            return None
        if spec.name not in _LOCO_SKILLS:
            raise RobotSkillError(f"G1 has no skill {spec.name!r}")
        self._end_turn()  # a new locomotion command replaces a running turn
        loco = self._loco_client()
        kind, value = _LOCO_SKILLS[spec.name][1]
        if kind == "turn":
            self._start_turn(loco, float(args["degrees"]))
            return self.turn_estimate_s(float(args["degrees"]))
        if kind == "walk":
            code = loco.SetVelocity(args["vx"], args["vy"], args["vyaw"], args["seconds"])
        elif kind == "height":
            code = loco.SetStandHeight(value)
        else:
            code = loco.SetFsmId(value)
        _check(spec.name, code)
        return float(args["seconds"]) if kind == "walk" else None

    def turn_estimate_s(self, degrees: float) -> float:
        """Upper estimate of how long ``turn(degrees)`` runs, seconds.

        The time the loop needs at the commanded rates (full speed, then
        slowing near the goal), divided by 0.7 because the real G1 turned about
        0.7 of the commanded rate under ``walk``, plus 0.5 s; never more than
        ``turn_timeout``. 0 inside the tolerance, where nothing is sent.
        """
        goal = abs(math.radians(degrees))
        tol = math.radians(TURN_LIMITS["tolerance"])
        if goal <= tol:
            return 0.0
        fast, slow, k = TURN_LIMITS["vyaw_max"], TURN_LIMITS["vyaw_min"], 1.5
        hi, lo = fast / k, slow / k  # rate is fast above hi, k * r between, slow below lo
        t = max(goal - hi, 0.0) / fast
        t += math.log(min(goal, hi) / lo) / k if goal > lo else 0.0
        t += (min(goal, lo) - tol) / slow
        return min(t / 0.7 + 0.5, self._turn_timeout)

    def stop(self) -> None:
        """Stop walking (and a running turn) and release the arms; best effort, never raises."""
        def release_arm() -> None:
            _check("release arm", self._arm_action("release arm", _RELEASE_ARM))

        self._end_turn()
        for what, call in (("StopMove", lambda: self._loco_client().StopMove()),
                           ("release arm", release_arm)):
            try:
                call()
            except Exception as exc:  # noqa: BLE001 - stop must try every path
                logger.warning("G1 skill stop: %s failed: %s", what, exc)

    def _start_turn(self, loco: Any, degrees: float) -> None:
        """Send the first turn command here (so a refusal raises), then close the loop on a thread."""
        if self._yaw is None:
            raise RobotSkillError("turn needs the IMU yaw, which this G1Skills was not given")
        if self._yaw() is None:
            raise RobotSkillError("turn: no fresh IMU reading from the robot")
        goal = math.radians(degrees)
        if abs(goal) <= math.radians(TURN_LIMITS["tolerance"]):
            return
        _check("turn", loco.SetVelocity(0.0, 0.0, self._turn_rate(goal), TURN_LIMITS["command_s"]))
        cancel = threading.Event()
        thread = threading.Thread(target=self._turn_loop, args=(loco, goal, cancel),
                                  name="g1-turn", daemon=True)
        self._turn_cancel, self._turn_thread = cancel, thread
        thread.start()

    @staticmethod
    def _turn_rate(remaining: float) -> float:
        """Turn rate for ``remaining`` radians: slows down near the target."""
        speed = min(TURN_LIMITS["vyaw_max"], max(TURN_LIMITS["vyaw_min"], 1.5 * abs(remaining)))
        return math.copysign(speed, remaining)

    def _turn_loop(self, loco: Any, goal: float, cancel: threading.Event) -> None:
        tolerance = math.radians(TURN_LIMITS["tolerance"])
        last = self._yaw()
        turned = 0.0
        end = time.monotonic() + self._turn_timeout
        why = "reached"
        try:
            while True:
                if cancel.wait(self._turn_period):
                    why = "stopped"
                    break
                yaw = self._yaw()
                if yaw is None or last is None:
                    why = "lost the IMU reading"
                    break
                # Sum the small steps, so turns past 180 degrees in total stay right.
                turned += math.remainder(yaw - last, math.tau)
                last = yaw
                remaining = goal - turned
                if abs(remaining) <= tolerance:
                    break
                if time.monotonic() > end:
                    why = f"timed out after {self._turn_timeout:.0f} s"
                    break
                code = loco.SetVelocity(0.0, 0.0, self._turn_rate(remaining),
                                        TURN_LIMITS["command_s"])
                if code not in (0, None):
                    why = f"refused (RPC code {code})"
                    break
        except Exception as exc:  # noqa: BLE001 - the loop must always stop the robot
            why = f"failed: {exc}"
        finally:
            try:
                loco.StopMove()
            except Exception as exc:  # noqa: BLE001
                logger.warning("G1 turn: StopMove failed: %s", exc)
        log = logger.info if why in ("reached", "stopped") else logger.warning
        log("G1 turn %s: asked %.1f deg, turned %.1f deg", why,
            math.degrees(goal), math.degrees(turned))

    def _end_turn(self) -> None:
        """Cancel a running turn and wait until its loop has stopped the robot."""
        cancel, thread = self._turn_cancel, self._turn_thread
        self._turn_cancel = self._turn_thread = None
        if cancel is None or thread is None:
            return
        cancel.set()
        if thread is not threading.current_thread():
            thread.join(self._timeout + 1.0)

    def _arm_action(self, name: str, action_id: int) -> Any:
        """``ExecuteAction`` on a thread; its code if it replies within ``accept_s``, else None."""
        arm = self._arm_client()
        done, late = threading.Event(), threading.Event()
        result: List[Any] = []

        def call() -> None:
            try:
                result.append(arm.ExecuteAction(action_id))
            except Exception as exc:  # noqa: BLE001 - reported below or logged
                result.append(exc)
            done.set()
            if late.is_set() and result[0] not in (0, None):
                logger.warning("G1 gesture %s ended with %r after run() had returned",
                               name, result[0])

        threading.Thread(target=call, name=f"g1-arm-{name}", daemon=True).start()
        if not done.wait(self._accept_s):
            late.set()
            if not done.is_set():
                return None  # still running: accepted
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]


def _check(name: str, code: Any) -> None:
    if code not in (0, None):
        why = ("built-in motion service not running (select 'ai' first)"
               if code == _RPC_ERR_CLIENT_SEND else "refused")
        raise RobotSkillError(f"onboard skill {name}: {why} (RPC code {code})")


def _sdk_loco_client():
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    return LocoClient()


def _sdk_arm_client():
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

    return G1ArmActionClient()
