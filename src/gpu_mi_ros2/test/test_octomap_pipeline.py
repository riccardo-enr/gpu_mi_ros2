"""Structural tests for the OctoMap pipeline (issue #9).

Verifies that:
- config/octomap.yaml has the required keys for the agreed indoor RGBD sensor model.
- launch/octomap.launch.py spawns octomap_server_node with cloud_in remapped to
  /camera/depth/points, and a static_transform_publisher anchoring map -> odom
  (identity, since DiffDrive sim odom is exact).
- sim.launch.py exposes a `mode` argument and includes octomap.launch.py when
  mode == 3d.
- package.xml declares octomap_server + octomap_msgs.

Proper world-pose ground truth (via gz /world/<w>/dynamic_pose/info) is tracked
as a follow-up tied to the UAV milestone -- in sim, DiffDrive odom is exact so
the residual map -> odom is identity and the static anchor is sufficient.
"""
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


PKG_DIR = Path(__file__).resolve().parents[1]
ROBOT_SDF = PKG_DIR / "models" / "demo_robot" / "model.sdf"
OCTOMAP_YAML = PKG_DIR / "config" / "octomap.yaml"
OCTOMAP_LAUNCH = PKG_DIR / "launch" / "octomap.launch.py"
SIM_LAUNCH = PKG_DIR / "launch" / "sim.launch.py"
PACKAGE_XML = PKG_DIR / "package.xml"


def _load_module(path: Path, name: str):
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


def _node_arg_text(node: Node) -> str:
    parts = []
    for token in node.cmd or []:
        if isinstance(token, list):
            for s in token:
                if isinstance(s, TextSubstitution):
                    parts.append(s.text)
                else:
                    parts.append(repr(s))
        elif isinstance(token, TextSubstitution):
            parts.append(token.text)
        else:
            parts.append(str(token))
    return " ".join(parts)


# ----- octomap.yaml ---------------------------------------------------------


def test_octomap_yaml_has_required_keys():
    assert OCTOMAP_YAML.is_file(), f"missing {OCTOMAP_YAML}"
    with OCTOMAP_YAML.open() as f:
        cfg = yaml.safe_load(f)

    # octomap_server uses ROS 2 nested params layout.
    assert "octomap_server" in cfg, "top-level octomap_server key required"
    params = cfg["octomap_server"]["ros__parameters"]

    assert float(params["resolution"]) == pytest.approx(0.1)
    assert params["frame_id"] == "map"
    assert params["base_frame_id"] == "base_link"
    assert float(params["sensor_model.max_range"]) == pytest.approx(5.0)
    assert float(params["sensor_model.hit"]) == pytest.approx(0.7)
    assert float(params["sensor_model.miss"]) == pytest.approx(0.4)
    assert float(params["sensor_model.min"]) == pytest.approx(0.12)
    assert float(params["sensor_model.max"]) == pytest.approx(0.97)
    assert float(params["occupancy_min_z"]) == pytest.approx(0.05)
    # Latched so Nav2's global_costmap static layer (transient_local subscriber)
    # gets the latest /projected_map on late-join. See issue #11.
    assert params["latch"] is True
    assert params["publish_free_space"] is False


# ----- octomap.launch.py ----------------------------------------------------


def test_octomap_launch_module_importable():
    assert OCTOMAP_LAUNCH.is_file(), f"missing {OCTOMAP_LAUNCH}"
    mod = _load_module(OCTOMAP_LAUNCH, "octomap_launch_under_test")
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_octomap_launch_spawns_server_with_cloud_remap():
    mod = _load_module(OCTOMAP_LAUNCH, "octomap_launch_server")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    server_nodes = []
    for n in nodes:
        pkg = "".join(str(s) for s in (n.node_package or []))
        exe = "".join(str(s) for s in (n.node_executable or []))
        if pkg == "octomap_server" and exe == "octomap_server_node":
            server_nodes.append(n)
    assert len(server_nodes) == 1, "expected exactly one octomap_server_node"

    server = server_nodes[0]
    remappings = getattr(server, "_Node__remappings", []) or []
    remap_text_parts = []
    for entry in remappings:
        for side in entry:
            if isinstance(side, (list, tuple)):
                for s in side:
                    if isinstance(s, TextSubstitution):
                        remap_text_parts.append(s.text)
                    else:
                        remap_text_parts.append(str(s))
            elif isinstance(side, TextSubstitution):
                remap_text_parts.append(side.text)
            else:
                remap_text_parts.append(str(side))
    remap_text = " ".join(remap_text_parts)
    assert "cloud_in" in remap_text and "/camera/depth/points" in remap_text, (
        f"octomap_server_node must remap cloud_in -> /camera/depth/points; got: {remap_text}"
    )


def test_octomap_launch_publishes_static_map_to_odom():
    mod = _load_module(OCTOMAP_LAUNCH, "octomap_launch_static_tf")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    static_tfs = []
    for n in nodes:
        pkg = "".join(str(s) for s in (n.node_package or []))
        exe = "".join(str(s) for s in (n.node_executable or []))
        if pkg == "tf2_ros" and exe == "static_transform_publisher":
            text = " ".join(
                s.text if isinstance(s, TextSubstitution) else str(s)
                for token in (n.cmd or [])
                for s in (token if isinstance(token, list) else [token])
            )
            if "map" in text and "odom" in text:
                static_tfs.append(text)
    assert static_tfs, (
        "octomap.launch.py must spawn a static_transform_publisher anchoring "
        "map -> odom (identity)"
    )


# ----- sim.launch.py mode arg -----------------------------------------------


def test_sim_launch_exposes_mode_argument():
    mod = _load_module(SIM_LAUNCH, "sim_launch_mode_arg")
    ld = mod.generate_launch_description()
    from launch.actions import DeclareLaunchArgument

    decls = [e for e in _flatten(ld.entities) if isinstance(e, DeclareLaunchArgument)]
    names = {d.name for d in decls}
    assert "mode" in names, "sim.launch.py must expose a `mode` LaunchArgument"


def test_sim_launch_includes_octomap_when_3d():
    mod = _load_module(SIM_LAUNCH, "sim_launch_octomap_include")
    ld = mod.generate_launch_description()
    includes = [
        e for e in _flatten(ld.entities) if isinstance(e, IncludeLaunchDescription)
    ]
    matched = any(
        "octomap.launch.py" in str(inc.launch_description_source.location)
        for inc in includes
    )
    assert matched, "sim.launch.py must IncludeLaunchDescription(octomap.launch.py)"


# ----- package.xml ----------------------------------------------------------


def test_package_xml_declares_octomap_deps():
    tree = ET.parse(PACKAGE_XML)
    root = tree.getroot()
    deps = {e.text for e in root.findall("exec_depend")}
    deps |= {e.text for e in root.findall("depend")}
    assert "octomap_server" in deps
    assert "octomap_msgs" in deps


