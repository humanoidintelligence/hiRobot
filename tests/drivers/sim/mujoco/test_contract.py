"""``mujoco`` passes the body contract on a three-joint arm (needs the ``sim`` extra)."""

import pytest

from hirobot import Embodiment, get_body
from tests.drivers.contract import check_sim

pytest.importorskip("mujoco")

ARM_XML = """
<mujoco>
  <worldbody>
    <body name="link0" pos="0 0 0.1">
      <joint name="j0" axis="0 0 1"/>
      <geom size="0.05" mass="1"/>
      <body name="link1" pos="0 0 0.3">
        <joint name="j1" axis="0 1 0"/>
        <geom size="0.05" mass="1"/>
        <body name="link2" pos="0 0 0.3">
          <joint name="j2" axis="0 1 0"/>
          <geom size="0.05" mass="1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j0"/>
    <motor joint="j1"/>
    <motor joint="j2"/>
  </actuator>
</mujoco>
"""


def test_contract(tmp_path):
    path = tmp_path / "arm.xml"
    path.write_text(ARM_XML)
    emb = Embodiment(name="arm", kind="arm", floating_base=False, control_hz=50,
                     joints=[{"name": f"j{i}"} for i in range(3)])
    made = []

    def make():
        robot = get_body("mujoco")(emb, model_path=str(path), realtime=False)
        made.append(robot)
        return robot

    def sent():
        # Simulated time advances only when a command is applied.
        return sum(r.data.time for r in made if r.data is not None)

    check_sim("mujoco", make, sent)
