"""Layers that ship with hiRobot (see :class:`~hirobot.control.layer.Layer`).

- ``mink_ik`` (:class:`~hirobot.layers.mink_ik.MinkIkLayer`): ``CARTESIAN`` and
  ``CARTESIAN_DELTA`` end-effector targets turned into joint ``POSITION``
  targets by differential IK (mink). Needs ``pip install 'hirobot[ik]'``.

Importing this package registers them; :func:`~hirobot.loading.registry.load_drivers`
does that with the built-in drivers. The optional dependencies are imported only
when a layer is built.
"""

from hirobot.layers.mink_ik import MinkIkLayer

__all__ = ["MinkIkLayer"]
