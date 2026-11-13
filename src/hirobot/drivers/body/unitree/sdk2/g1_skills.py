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
None of these skills has been run on a robot yet.
"""

from __future__ import annotations

import logging
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


def _num(desc: str, limit: float) -> Dict[str, Any]:
    return {"type": "number", "minimum": -limit, "maximum": limit, "default": 0.0,
            "description": desc}


def _spec(name: str, description: str, requires: List[str], **kw: Any) -> RobotSkillSpec:
    return RobotSkillSpec(name=name, description=description, requires=requires, **kw)


#: Locomotion skills: name -> (spec, how it is sent). The second item is
#: ("fsm", id), ("height", value) or ("walk", None).
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
        timeout: RPC timeout, seconds.
    """

    def __init__(
        self,
        loco_client: Optional[Callable[[], Any]] = None,
        arm_client: Optional[Callable[[], Any]] = None,
        timeout: float = 3.0,
    ) -> None:
        self._make_loco = loco_client or _sdk_loco_client
        self._make_arm = arm_client or _sdk_arm_client
        self._timeout = timeout
        self._loco = None
        self._arm = None

    def _loco_client(self):
        if self._loco is None:
            self._loco = self._make_loco()
            self._loco.SetTimeout(self._timeout)
            self._loco.Init()
        return self._loco

    def _arm_client(self):
        if self._arm is None:
            self._arm = self._make_arm()
            self._arm.SetTimeout(self._timeout)
            self._arm.Init()
        return self._arm

    def run(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> None:
        if spec.name in GESTURES:
            _check(spec.name, self._arm_client().ExecuteAction(GESTURES[spec.name][0]))
            return
        if spec.name not in _LOCO_SKILLS:
            raise RobotSkillError(f"G1 has no skill {spec.name!r}")
        loco = self._loco_client()
        kind, value = _LOCO_SKILLS[spec.name][1]
        if kind == "walk":
            code = loco.SetVelocity(args["vx"], args["vy"], args["vyaw"], args["seconds"])
        elif kind == "height":
            code = loco.SetStandHeight(value)
        else:
            code = loco.SetFsmId(value)
        _check(spec.name, code)

    def stop(self) -> None:
        """Stop walking and release the arms; best effort, never raises."""
        for what, call in (("StopMove", lambda: self._loco_client().StopMove()),
                           ("release arm", lambda: self._arm_client().ExecuteAction(_RELEASE_ARM))):
            try:
                call()
            except Exception as exc:  # noqa: BLE001 - stop must try every path
                logger.warning("G1 skill stop: %s failed: %s", what, exc)


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
