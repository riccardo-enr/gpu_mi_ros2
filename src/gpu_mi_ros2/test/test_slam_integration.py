"""Structural tests for slam_toolbox integration (issue #5).

Verifies the package declares the slam_toolbox runtime dependency, ships a
valid params YAML with the frames matching the diff-drive plugin, exposes a
slam.launch.py that spawns async_slam_toolbox_node, and that sim.launch.py
includes slam.launch.py so a single `pixi run sim` brings the SLAM stack up.
"""
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


PKG_DIR = Path(__file__).resolve().parents[1]
PACKAGE_XML = PKG_DIR / "package.xml"
SLAM_PARAMS = PKG_DIR / "config" / "slam_toolbox.yaml"
SLAM_LAUNCH = PKG_DIR / "launch" / "slam.launch.py"
SIM_LAUNCH = PKG_DIR / "launch" / "sim.launch.py"


def _load_launch_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _flatten(entities):
    """Walk LaunchDescription entities, descending into TimerAction.actions."""
    out = []
    for e in entities:
        out.append(e)
        if isinstance(e, TimerAction):
            out.extend(_flatten(e.actions))
    return out


def test_slam_toolbox_in_package_xml():
    tree = ET.parse(PACKAGE_XML)
    exec_deps = {e.text for e in tree.getroot().findall("exec_depend")}
    depends = {e.text for e in tree.getroot().findall("depend")}
    assert "slam_toolbox" in exec_deps | depends, (
        "package.xml must declare slam_toolbox as exec_depend so rosdep "
        "installs ros-jazzy-slam-toolbox"
    )


def test_slam_params_yaml_exists_and_valid():
    assert SLAM_PARAMS.is_file(), f"missing {SLAM_PARAMS}"
    with SLAM_PARAMS.open() as f:
        cfg = yaml.safe_load(f)

    # slam_toolbox uses the standard ROS 2 nested params layout:
    # slam_toolbox: { ros__parameters: {...} }
    assert "slam_toolbox" in cfg, "top-level 'slam_toolbox' key required"
    params = cfg["slam_toolbox"]["ros__parameters"]

    assert params["mode"] == "mapping"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_link"
    assert params["map_frame"] == "map"
    assert params["scan_topic"] == "/scan"
    assert params["use_sim_time"] is True
    assert float(params["max_laser_range"]) == pytest.approx(12.0)


def test_slam_launch_module_importable_and_spawns_async_node():
    assert SLAM_LAUNCH.is_file(), f"missing {SLAM_LAUNCH}"
    mod = _load_launch_module(SLAM_LAUNCH, "slam_launch_under_test")
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]
    assert len(nodes) == 1, f"expected exactly one Node, got {len(nodes)}"
    node = nodes[0]

    # Node stores package/executable as substitutions; str() collapses literals.
    pkg = "".join(str(s) for s in (node.node_package or []))
    exe = "".join(str(s) for s in (node.node_executable or []))
    assert "slam_toolbox" in pkg
    assert "async_slam_toolbox_node" in exe


def test_sim_launch_includes_slam():
    mod = _load_launch_module(SIM_LAUNCH, "sim_launch_under_test")
    ld = mod.generate_launch_description()
    includes = [
        e for e in _flatten(ld.entities) if isinstance(e, IncludeLaunchDescription)
    ]
    assert includes, "sim.launch.py must IncludeLaunchDescription(slam.launch.py)"
    matched = any(
        "slam.launch.py" in str(getattr(inc, "_get_launch_file", lambda: "")() or "")
        or "slam.launch.py" in repr(inc.launch_description_source)
        for inc in includes
    )
    assert matched, "no IncludeLaunchDescription points at slam.launch.py"
