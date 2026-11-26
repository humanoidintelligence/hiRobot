"""FrameClock (hirobot.sim.sim_world): when a simulated camera renders. No MuJoCo needed."""

import numpy as np
import pytest

from hirobot.sim.sim_world import FrameClock, camera_fps_option, sim_options


def _run(clock, times):
    renders = []

    def render():
        renders.append(1)
        return np.zeros((2, 2, 3), dtype=np.uint8)

    return [clock.frame(t, render) for t in times], renders


def test_renders_on_the_fps_grid_of_simulated_time():
    frames, renders = _run(FrameClock(30.0), [k * 0.02 for k in range(50)])
    assert len(renders) == 30
    assert [f[1].seq for f in frames[:8]] == [1, 1, 2, 2, 3, 4, 4, 5]
    assert frames[1][0] is frames[0][0] and frames[1][1] == frames[0][1]


def test_none_renders_every_call():
    frames, renders = _run(FrameClock(None), [0.0, 0.0, 0.02])
    assert len(renders) == 3 and [f[1].seq for f in frames] == [1, 2, 3]


def test_restart_renders_on_the_next_call_and_keeps_counting():
    clock = FrameClock(1.0)
    _, renders = _run(clock, [0.0, 0.1, 0.2])
    assert len(renders) == 1
    clock.restart()
    frames, renders = _run(clock, [0.3, 0.4])
    assert len(renders) == 1 and frames[0][1] == (0.3, 2) and frames[1][1] == (0.3, 2)


def test_time_going_back_renders():
    frames, renders = _run(FrameClock(10.0), [1.0, 1.05, 0.5, 0.55])
    assert len(renders) == 2 and frames[2][1] == (0.5, 2) and frames[3][1] == (0.5, 2)


def test_no_renderer_gives_none():
    assert FrameClock(30.0).frame(0.0, lambda: None) is None


def test_camera_fps_option_and_sim_section():
    assert camera_fps_option(None) is None and camera_fps_option(15) == 15.0
    for bad in (0, -1, True, "30", float("inf")):
        with pytest.raises(ValueError, match="positive number or null"):
            camera_fps_option(bad)
    assert sim_options({})["camera_fps"] == 30.0
    assert sim_options({"camera_fps": None})["camera_fps"] is None
    with pytest.raises(ValueError, match="sim.camera_fps"):
        sim_options({"camera_fps": 0})
