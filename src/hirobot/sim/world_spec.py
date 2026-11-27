"""The composed MuJoCo spec of a robot's bodies, for drawing or kinematics only.

:func:`world_spec` attaches every body's model file the way the shared simulation
world does (:class:`~hirobot.sim.sim_world.SimWorld`): each at its resolved
placement, its names prefixed with its node's path (``left/shoulder_pan_joint``),
its ``model.joint_names`` renames applied, a ``fixed_base`` body's free joint removed
and its root raised to ``base_height``.
It adds nothing else: no default stage (floor, light, sky), no motors, no cameras.
Nothing is connected or simulated; a viewer can compile the spec and run forward
kinematics on it, matching the robot's joint names as the simulation driver does
(``name`` or ``name_joint``).
"""

from __future__ import annotations

import os
from typing import Any, List, Mapping, Optional, Tuple, Union

import numpy as np

from hirobot.geometry.transforms import IDENTITY_POSE
from hirobot.sim.sim_world import WORLD, WorldBody, build_world, sim_options


def _wxyz(q: Any) -> List[float]:
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def _prepare(mj: Any, spec: Any, body: WorldBody) -> None:
    """What the world does to a body's own spec before attaching it (renames, ``fixed_base``,
    height)."""
    names = getattr(body, "joint_names", None)
    if names:
        from hirobot.sim.sim_world import rename_joints

        rename_joints(mj, spec, names, body.mjcf)
    free = [j for j in spec.joints if j.type == mj.mjtJoint.mjJNT_FREE]
    root = free[0].parent if free else None
    if body.fixed_base:
        for j in free:
            spec.delete(j)
    if body.height is not None and root is not None:
        root.pos = [root.pos[0], root.pos[1], float(body.height)]


def world_spec(config: Union[str, "os.PathLike[str]", Mapping[str, Any]],
               sim: Optional[Mapping[str, Any]] = None) -> Tuple[Any, List[str]]:
    """The ``mujoco.MjSpec`` of every body of ``config``, and its joint names in that spec.

    Args:
        config: A built-in config name, a path, or a loaded dict (an ``assembly:`` or
            ``device:`` config, as :func:`hirobot.loading.tree.load_robot` takes).
        sim: A ``sim:`` section to place the bodies with (``fixed_base``,
            ``base_height``); default the config's own, else none.

    Returns:
        ``(spec, joints)``: the uncompiled spec, and the names of its joints in spec
        order (``left/shoulder_pan_joint``, ``right/joint1``; a named free joint included,
        an unnamed one left out).

    Raises:
        ValueError: The config does not load, or a body has no ``model:``.
        FileNotFoundError: A body's model file is not there.
    """
    from hirobot.loading.layout import node_layout
    from hirobot.loading.tree import load_robot
    from hirobot.sim.sim_world import _copy_option, _import_mujoco

    mj = _import_mujoco()
    spec_tree = load_robot(config)
    options = sim_options(spec_tree.sim if sim is None else sim)
    bodies = build_world(spec_tree, node_layout(spec_tree.root), options).world.bodies
    for b in bodies:
        if not os.path.isfile(b.mjcf):
            from hirobot.assets import fetch_hint

            raise FileNotFoundError(
                f"model file of {b.path} not found: {b.mjcf}. {fetch_hint(b.mjcf)}")
    alone = len(bodies) == 1 and not bodies[0].prefix and bodies[0].anchor == WORLD \
        and np.array_equal(bodies[0].pose, IDENTITY_POSE)
    if alone:
        spec = mj.MjSpec.from_file(bodies[0].mjcf)
        _prepare(mj, spec, bodies[0])
    else:
        spec = mj.MjSpec()
        for i, b in enumerate(bodies):
            child = mj.MjSpec.from_file(b.mjcf)
            # As the world does: the first body's options, so attaching raises no conflicts.
            if i == 0:
                _copy_option(child, spec)
            else:
                _copy_option(spec, child)
            _prepare(mj, child, b)
            anchor = spec.worldbody if b.anchor == WORLD else spec.body(b.anchor)
            if anchor is None:
                raise ValueError(f"{b.path}: it is placed on {b.anchor!r}, which is not a body "
                                 "of the world")
            frame = anchor.add_frame(pos=[float(x) for x in b.pose[:3]], quat=_wxyz(b.pose[3:]))
            spec.attach(child, prefix=b.prefix, frame=frame)
    return spec, [j.name for j in spec.joints if j.name]


__all__ = ["world_spec"]
