"""Structural tests for the OctoMap pipeline (issue #9).

Verifies that:
- demo_robot SDF carries a PosePublisher plugin so world-frame pose flows into /tf.
- A gt_pose_to_tf module exists and exposes a pure compute_map_to_odom(map_T_base,
  odom_T_base) helper that returns map_T_base @ inv(odom_T_base).
- config/octomap.yaml has the required keys for the agreed indoor RGBD sensor model.
- launch/octomap.launch.py spawns octomap_server_node with cloud_in remapped to
  /camera/depth/points, and spawns the gt_pose_to_tf node.
- sim.launch.py exposes a `mode` argument and includes octomap.launch.py when
  mode == 3d.
- package.xml declares octomap_server + octomap_msgs.
- setup.py registers the gt_pose_to_tf console_script.
"""
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
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
SETUP_PY = PKG_DIR / "setup.py"
GT_POSE_MODULE = PKG_DIR / "gpu_mi_ros2" / "gt_pose_to_tf.py"


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


# ----- SDF: PosePublisher plugin --------------------------------------------


def test_sdf_has_pose_publisher_plugin():
    tree = ET.parse(ROBOT_SDF)
    root = tree.getroot()
    model = root.find("model") if root.tag == "sdf" else root
    pose_publishers = []
    for plugin in model.findall("plugin"):
        if "PosePublisher" in (plugin.get("name") or ""):
            pose_publishers.append(plugin)
    assert len(pose_publishers) == 1, (
        "demo_robot SDF must declare exactly one PosePublisher plugin"
    )
    plugin = pose_publishers[0]
    publish_model = plugin.find("publish_model_pose")
    assert publish_model is not None and publish_model.text.strip().lower() in (
        "true",
        "1",
    ), "PosePublisher must enable publish_model_pose"


# ----- gt_pose_to_tf residual math ------------------------------------------


def _se3(x=0.0, y=0.0, z=0.0, yaw=0.0):
    """Return a 4x4 SE(3) for 2D pose (yaw rotation in xy)."""
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T[:3, 3] = (x, y, z)
    return T


def test_gt_pose_compute_map_to_odom_identity():
    assert GT_POSE_MODULE.is_file(), f"missing {GT_POSE_MODULE}"
    mod = _load_module(GT_POSE_MODULE, "gt_pose_to_tf_under_test")
    assert hasattr(mod, "compute_map_to_odom"), (
        "gt_pose_to_tf must expose a pure compute_map_to_odom(map_T_base, odom_T_base)"
    )
    # When map_T_base == odom_T_base (sim case with perfect odom), residual is identity.
    base = _se3(1.0, 2.0, 0.0, np.pi / 4)
    out = mod.compute_map_to_odom(base, base)
    np.testing.assert_allclose(out, np.eye(4), atol=1e-12)


def test_gt_pose_compute_map_to_odom_offset():
    mod = _load_module(GT_POSE_MODULE, "gt_pose_to_tf_offset")
    map_T_base = _se3(3.0, 0.0, 0.0, 0.0)
    odom_T_base = _se3(1.0, 0.0, 0.0, 0.0)
    out = mod.compute_map_to_odom(map_T_base, odom_T_base)
    expected = map_T_base @ np.linalg.inv(odom_T_base)
    np.testing.assert_allclose(out, expected, atol=1e-12)
    # Translation should be (2, 0, 0).
    np.testing.assert_allclose(out[:3, 3], [2.0, 0.0, 0.0], atol=1e-12)


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
    assert params["latch"] is False
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
    # remappings is a list of (src, dst) tuples in launch_ros.
    remap_text = repr(getattr(server, "_Node__remappings", "")) + repr(
        getattr(server, "expanded_remapping_rules", "")
    )
    assert "cloud_in" in remap_text and "/camera/depth/points" in remap_text, (
        "octomap_server_node must remap cloud_in -> /camera/depth/points"
    )


def test_octomap_launch_spawns_gt_pose_to_tf():
    mod = _load_module(OCTOMAP_LAUNCH, "octomap_launch_gt")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    gt_nodes = []
    for n in nodes:
        pkg = "".join(str(s) for s in (n.node_package or []))
        exe = "".join(str(s) for s in (n.node_executable or []))
        if pkg == "gpu_mi_ros2" and exe == "gt_pose_to_tf":
            gt_nodes.append(n)
    assert gt_nodes, (
        "octomap.launch.py must spawn gpu_mi_ros2/gt_pose_to_tf"
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


# ----- setup.py entry point -------------------------------------------------


def test_setup_py_registers_gt_pose_to_tf_console_script():
    text = SETUP_PY.read_text()
    assert "gt_pose_to_tf" in text, (
        "setup.py must register a gt_pose_to_tf console_script entry"
    )
    assert "gpu_mi_ros2.gt_pose_to_tf" in text, (
        "console_script must point at gpu_mi_ros2.gt_pose_to_tf:main"
    )
