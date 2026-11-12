"""Config loading: built-in names, ``defaults:`` composition and the merge rules."""

from pathlib import Path

import pytest
import yaml

from hirobot import load_embodiment, make_robot
from hirobot.loading.config import available_configs, load_config, merge, resolve


def test_dicts_merge_deeply_and_other_lists_are_replaced():
    base = {"body": {"driver": "toy", "gains": [1, 2], "opts": {"a": 1}}}
    over = {"body": {"gains": [3], "opts": {"b": 2}}}
    assert merge(base, over) == {"body": {"driver": "toy", "gains": [3], "opts": {"a": 1, "b": 2}}}
    assert base["body"]["gains"] == [1, 2]  # inputs are not modified


def test_named_lists_merge_by_name_and_append_new_names():
    body = {"joints": [{"name": "hip"}, {"name": "knee"}],
            "end_effectors": [{"name": "left_hand"}, {"name": "right_hand"}]}
    hand = {"joints": [{"name": "knee", "torque_limit": 9.0}, {"name": "finger"}],
            "end_effectors": [{"name": "left_hand", "driver": {"type": "x"}}]}
    out = merge(body, hand)
    assert [j["name"] for j in out["joints"]] == ["hip", "knee", "finger"]
    assert out["joints"][1] == {"name": "knee", "torque_limit": 9.0}
    assert out["end_effectors"][0] == {"name": "left_hand", "driver": {"type": "x"}}
    assert out["end_effectors"][1] == {"name": "right_hand"}


def test_driver_spec_is_replaced_when_its_type_changes():
    base = {"speakers": [{"name": "head", "driver": {"type": "unitree_voice_speaker", "volume": 60}}]}
    other = {"speakers": [{"name": "head", "driver": {"type": "null_speaker"}}]}
    same = {"speakers": [{"name": "head", "driver": {"type": "unitree_voice_speaker", "volume": 30}}]}
    assert merge(base, other)["speakers"][0]["driver"] == {"type": "null_speaker"}
    assert merge(base, same)["speakers"][0]["driver"] == {"type": "unitree_voice_speaker", "volume": 30}
    # an override without a type still merges into the existing driver
    assert merge(base, {"speakers": [{"name": "head", "driver": {"volume": 10}}]})[
        "speakers"][0]["driver"] == {"type": "unitree_voice_speaker", "volume": 10}


def test_simulated_g1_takes_stand_in_audio(tmp_path):
    wav = tmp_path / "in.wav"
    import wave
    with wave.open(str(wav), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b"\0\0" * 160)
    cfg = tmp_path / "g1.yaml"
    cfg.write_text(
        "defaults: [unitree_g1]\n"
        "assembly:\n"
        "  parts:\n"
        f"    head_mic: {{overrides: {{driver: {{type: wav_mic, path: {wav}}}}}}}\n"
        "    head_speaker: {overrides: {driver: {type: null_speaker}}}\n")
    robot = make_robot(str(cfg))  # body is the toy driver, so only stand-in devices are built
    assert type(robot.microphones["head_mic"]).__name__ == "WavMicrophone"
    assert type(robot.speakers["head_speaker"]).__name__ == "NullSpeaker"


def test_named_list_entries_need_a_name():
    with pytest.raises(ValueError, match="needs a 'name'"):
        merge({"joints": [{"name": "a"}]}, {"joints": [{"torque_limit": 1}]})


def test_hand_joints_follow_the_body():
    emb = load_embodiment("unitree_g1_dex3")
    body = load_embodiment("body/unitree/g1")
    assert emb.joint_names[:29] == body.joint_names
    assert emb.joint_names[29] == "left_hand_thumb_0" and emb.num_dofs == 43
    assert emb.name == "unitree-g1-dex3"
    assert [c.name for c in emb.cameras] == ["head"]  # from robots/unitree_g1


def test_user_file_builds_on_builtin_parts_and_relative_files(tmp_path):
    (tmp_path / "gains.yaml").write_text(yaml.safe_dump(
        {"assembly": {"parts": {"body": {"overrides": {"driver": {"dt": 0.01}}}}}}))
    (tmp_path / "mine.yaml").write_text(
        "defaults: [unitree_h1, ./gains.yaml]\n"
        "assembly:\n"
        "  name: my-h1\n"
        "  parts: {body: {overrides: {driver: {type: toy}}}}\n")
    cfg = load_config(tmp_path / "mine.yaml")
    assert cfg["assembly"]["name"] == "my-h1"
    assert cfg["assembly"]["parts"]["body"]["overrides"] == {"driver": {"type": "toy", "dt": 0.01}}
    assert "defaults" not in cfg
    robot = make_robot(str(tmp_path / "mine.yaml"))
    assert robot.embodiment.num_dofs == 19 and robot.embodiment.name == "my-h1"


def test_defaults_cycle_is_reported(tmp_path):
    (tmp_path / "a.yaml").write_text("defaults: [./b.yaml]\n")
    (tmp_path / "b.yaml").write_text("defaults: [./a.yaml]\n")
    with pytest.raises(ValueError, match="cycle"):
        load_config(tmp_path / "a.yaml")


def test_names_paths_and_errors():
    assert resolve("unitree_g1").as_posix().endswith("hirobot/configs/robots/unitree_g1.yaml")
    assert resolve("robots/unitree_g1") == resolve("unitree_g1")
    assert resolve("body/unitree/h1").name == "h1.yaml"
    with pytest.raises(FileNotFoundError, match="hirobot configs"):
        resolve("unitree_g9")
    with pytest.raises(FileNotFoundError, match="config file not found"):
        resolve("./nope.yaml")


def test_make_robot_takes_a_dict():
    cfg = load_config("unitree_h1")
    cfg["assembly"]["name"] = "renamed"
    assert make_robot(cfg).embodiment.name == "renamed"


@pytest.mark.parametrize("name", available_configs())
def test_every_builtin_config_loads(name):
    cfg = load_config(name)
    if name.startswith(("robots/", "body/")):
        load_embodiment(cfg)


EXAMPLES = sorted((Path(__file__).parents[1] / "examples").glob("*.yaml"))


def test_examples_exist():
    assert len(EXAMPLES) >= 2


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.name)
def test_every_example_loads(path):
    # Examples are loaded by path; their defaults: mix built-in names and sibling files.
    assert load_embodiment(path).num_dofs > 0


def test_cli_lists_and_resolves(capsys):
    from hirobot.cli import main

    main(["configs", "end_effector"])
    assert capsys.readouterr().out.split() == [
        "end_effector/inspire/rh56dfx_left", "end_effector/inspire/rh56dfx_right",
        "end_effector/robotiq/2f85_urcap",
        "end_effector/unitree/dex3_1_left", "end_effector/unitree/dex3_1_right"]
    main(["info", "--config", "unitree_g1", "--resolved"])
    out = capsys.readouterr().out
    assert yaml.safe_load(out)["assembly"]["parts"]["head"] == {"from": "camera/intel/d435i"}


def test_expanded_configs_match_the_snapshot():
    # tests/data/expanded_configs.json is the expanded form of every whole-robot
    # config. Regenerate it only for a deliberate config change.
    import json
    from pathlib import Path

    snap = json.loads((Path(__file__).parent / "data" / "expanded_configs.json").read_text())
    names = available_configs("robots")
    assert sorted(snap) == names
    for name in names:
        assert json.loads(json.dumps(load_config(name))) == snap[name], name


def test_old_formats_are_rejected_or_deprecated():
    with pytest.raises(ValueError, match="group: section"):
        make_robot({"group": {"members": {}}})
    with pytest.raises(ValueError, match="body: section"):
        make_robot({"body": {"driver": "toy"}})
    old = {"embodiment": {"name": "old", "control_hz": 50, "joints": [{"name": "j"}]},
           "body": {"driver": "toy"}}
    with pytest.warns(DeprecationWarning, match="embodiment: config format"):
        assert make_robot(old).embodiment.name == "old"
    with pytest.warns(DeprecationWarning, match="body_overrides"):
        robot = make_robot("unitree_h1", body_overrides={"driver": "toy", "dt": 0.01})
    assert robot.embodiment.num_dofs == 19
