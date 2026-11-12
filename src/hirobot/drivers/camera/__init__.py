"""Camera drivers: frames for ``Observation.images``, independent of the body.

A camera is declared in the embodiment config with an optional ``driver:``;
``make_robot`` builds a :class:`CameraDriver` for each one that has it::

    cameras:
      - name: head
        width: 1920
        height: 1080
        driver:
          type: unitree_videohub     # a registered camera driver
          network_interface: eth0    # remaining keys go to the driver

Cameras without a ``driver:`` are rendered by the body driver itself (simulators,
e.g. MuJoCo or a benchmark environment). Vendor SDKs are imported lazily on ``start()``.

With a simulated body only camera drivers marked ``simulated = True`` are built
(``video_file``, ``array``); real cameras are skipped so the config runs offline.
"""

from hirobot.drivers.camera.agibot.cosine import AgibotCosineCamera
from hirobot.drivers.camera.generic import ArrayCamera, VideoFileCamera
from hirobot.drivers.camera.intel.realsense import RealSenseCamera
from hirobot.drivers.camera.unitree.image_server import UnitreeImageServerCamera
from hirobot.drivers.camera.unitree.sdk2_videohub import UnitreeVideohubCamera

__all__ = ["AgibotCosineCamera", "RealSenseCamera", "UnitreeVideohubCamera", "UnitreeImageServerCamera", "ArrayCamera", "VideoFileCamera"]
