"""Structural tests for wiring mi_field_node into the demo (issue #6).

Verifies that the demo params pin algo=fcmi at the demo publish rate, that
sim.launch.py includes mi_field.launch.py (so `pixi run sim` brings MI up),
and that mi_field.launch.py still spawns mi_field_node (regression guard).
"""
import importlib.util
from pathlib import Path

import pytest
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


PKG_DIR = Path(__file__).resolve().parents[1]
PARAMS_YAML = PKG_DIR / "config" / "params.yaml"
SIM_LAUNCH = PKG_DIR / "launch" / "sim.launch.py"
MI_LAUNCH = PKG_DIR / "launch" / "mi_field.launch.py"


def _load_launch_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _flatten(entities):
    out = []
    for e in entities:
        out.append(e)
        if isinstance(e, TimerAction):
            out.extend(_flatten(e.actions))
    return out


def test_params_yaml_algo_and_rate():
    with PARAMS_YAML.open() as f:
        cfg = yaml.safe_load(f)
    params = cfg["mi_field_node"]["ros__parameters"]
    assert params["algo"] == "fcmi"
    assert float(params["publish_rate"]) == pytest.approx(1.0)


def test_sim_launch_includes_mi_field():
    mod = _load_launch_module(SIM_LAUNCH, "sim_launch_mi_under_test")
    ld = mod.generate_launch_description()
    includes = [
        e for e in _flatten(ld.entities) if isinstance(e, IncludeLaunchDescription)
    ]
    matched = any(
        "mi_field.launch.py" in str(inc.launch_description_source.location)
        for inc in includes
    )
    assert matched, (
        "sim.launch.py must IncludeLaunchDescription(mi_field.launch.py)"
    )


def test_mi_launch_module_importable_and_spawns_mi_field_node():
    assert MI_LAUNCH.is_file(), f"missing {MI_LAUNCH}"
    mod = _load_launch_module(MI_LAUNCH, "mi_launch_under_test")
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]
    assert len(nodes) == 1, f"expected exactly one Node, got {len(nodes)}"
    node = nodes[0]
    pkg = "".join(str(s) for s in (node.node_package or []))
    exe = "".join(str(s) for s in (node.node_executable or []))
    assert "gpu_mi_ros2" in pkg
    assert "mi_field_node" in exe
