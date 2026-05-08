"""Structural tests for the Nav2 bringup (issue #11).

Verifies that:
- config/nav2.yaml has the required Nav2 server sections (planner, controller,
  costmaps, BT navigator, behavior server) wired for diff-drive demo_robot:
  global costmap consumes /projected_map (the 2D projection published by
  octomap_server), polygon footprint matches the chassis, conservative DWB
  velocity caps, SmacPlanner2D selected.
- launch/nav2.launch.py spawns planner_server, controller_server,
  bt_navigator, behavior_server, waypoint_follower plus a single
  nav2_lifecycle_manager (autostart=true) managing them.
- sim.launch.py exposes a `nav2` LaunchArgument (default false) and
  conditionally includes nav2.launch.py.
- package.xml declares the nav2_* exec_depends.
- setup.py installs config/nav2.yaml under share/.
"""
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


PKG_DIR = Path(__file__).resolve().parents[1]
NAV2_YAML = PKG_DIR / "config" / "nav2.yaml"
NAV2_LAUNCH = PKG_DIR / "launch" / "nav2.launch.py"
SIM_LAUNCH = PKG_DIR / "launch" / "sim.launch.py"
PACKAGE_XML = PKG_DIR / "package.xml"
SETUP_PY = PKG_DIR / "setup.py"


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


def _node_pkg_exe(n: Node):
    pkg = "".join(str(s) for s in (n.node_package or []))
    exe = "".join(str(s) for s in (n.node_executable or []))
    return pkg, exe


# ----- nav2.yaml -----------------------------------------------------------


def test_nav2_yaml_exists_and_parses():
    assert NAV2_YAML.is_file(), f"missing {NAV2_YAML}"
    with NAV2_YAML.open() as f:
        cfg = yaml.safe_load(f)
    assert isinstance(cfg, dict) and cfg, "nav2.yaml must be a non-empty mapping"


def test_nav2_yaml_has_required_servers():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    for key in (
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "global_costmap",
        "local_costmap",
    ):
        assert key in cfg, f"nav2.yaml missing top-level `{key}` block"


def test_nav2_yaml_uses_smac_planner_2d():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    params = cfg["planner_server"]["ros__parameters"]
    assert "GridBased" in params.get("planner_plugins", []), (
        "planner_server must list a `GridBased` planner plugin"
    )
    plugin = params["GridBased"]["plugin"]
    assert "nav2_smac_planner" in plugin and "SmacPlanner2D" in plugin, (
        f"GridBased plugin must be SmacPlanner2D, got {plugin!r}"
    )


def test_nav2_yaml_global_costmap_uses_projected_map():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    gc = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
    plugins = gc.get("plugins", [])
    assert "static_layer" in plugins, "global_costmap must include a static_layer"
    static = gc["static_layer"]
    assert static.get("map_topic") == "/projected_map", (
        "static_layer.map_topic must be /projected_map (octomap_server's 2D projection)"
    )


def test_nav2_yaml_polygon_footprint_matches_chassis():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    for scope in (
        cfg["global_costmap"]["global_costmap"]["ros__parameters"],
        cfg["local_costmap"]["local_costmap"]["ros__parameters"],
    ):
        fp = scope.get("footprint")
        assert fp, "costmap must declare a polygon `footprint` (not robot_radius)"
        # demo_robot chassis is 0.4 x 0.3 m; half-extents 0.2 x 0.15
        assert "0.2" in fp and "0.15" in fp, (
            f"footprint must reflect 0.4x0.3 chassis half-extents, got {fp!r}"
        )


def test_nav2_yaml_dwb_conservative_velocity_caps():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    cs = cfg["controller_server"]["ros__parameters"]
    plugins = cs.get("controller_plugins", [])
    assert "FollowPath" in plugins, "controller_server must list FollowPath"
    plugin = cs["FollowPath"]["plugin"]
    assert "dwb_core" in plugin and "DWBLocalPlanner" in plugin
    assert float(cs["FollowPath"]["max_vel_x"]) == pytest.approx(0.5)
    assert float(cs["FollowPath"]["max_vel_theta"]) == pytest.approx(1.0)


def test_nav2_yaml_local_costmap_obstacle_layer_uses_scan():
    cfg = yaml.safe_load(NAV2_YAML.read_text())
    lc = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "obstacle_layer" in lc.get("plugins", []), (
        "local_costmap must include an obstacle_layer"
    )
    obs = lc["obstacle_layer"]
    src_names = obs.get("observation_sources", "").split()
    assert src_names, "obstacle_layer.observation_sources must be set"
    src = obs[src_names[0]]
    assert src["topic"] == "/scan"
    assert src["data_type"] == "LaserScan"


# ----- nav2.launch.py ------------------------------------------------------


def test_nav2_launch_module_importable():
    assert NAV2_LAUNCH.is_file(), f"missing {NAV2_LAUNCH}"
    mod = _load_module(NAV2_LAUNCH, "nav2_launch_under_test")
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_nav2_launch_spawns_required_servers():
    mod = _load_module(NAV2_LAUNCH, "nav2_launch_servers")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]
    pkg_exe = {_node_pkg_exe(n) for n in nodes}

    expected = {
        ("nav2_planner", "planner_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_waypoint_follower", "waypoint_follower"),
    }
    missing = expected - pkg_exe
    assert not missing, f"nav2.launch.py missing nodes: {missing}"


def test_nav2_launch_has_lifecycle_manager_with_autostart():
    mod = _load_module(NAV2_LAUNCH, "nav2_launch_lifecycle")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    lcm = [n for n in nodes if _node_pkg_exe(n) == ("nav2_lifecycle_manager", "lifecycle_manager")]
    assert len(lcm) == 1, "expected exactly one nav2_lifecycle_manager"

    # Inspect the parameters dict for autostart=True and a non-empty node_names list.
    params_list = getattr(lcm[0], "_Node__parameters", []) or []
    flat = {}
    for entry in params_list:
        if isinstance(entry, dict):
            flat.update(entry)
    assert flat.get("autostart") is True, "lifecycle_manager.autostart must be True"
    nodes_managed = flat.get("node_names") or []
    for required in (
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
    ):
        assert required in nodes_managed, (
            f"lifecycle_manager must manage {required}, got {nodes_managed}"
        )


# ----- sim.launch.py nav2 arg ----------------------------------------------


def test_sim_launch_exposes_nav2_argument():
    mod = _load_module(SIM_LAUNCH, "sim_launch_nav2_arg")
    ld = mod.generate_launch_description()
    decls = [e for e in _flatten(ld.entities) if isinstance(e, DeclareLaunchArgument)]
    nav2_decls = [d for d in decls if d.name == "nav2"]
    assert nav2_decls, "sim.launch.py must expose a `nav2` LaunchArgument"
    assert nav2_decls[0].default_value, "nav2 arg must have a default"
    default_text = "".join(
        s.text if isinstance(s, TextSubstitution) else str(s)
        for s in nav2_decls[0].default_value
    )
    assert default_text == "false", "nav2 default must be 'false'"


def test_sim_launch_includes_nav2_launch():
    mod = _load_module(SIM_LAUNCH, "sim_launch_nav2_include")
    ld = mod.generate_launch_description()
    includes = [
        e for e in _flatten(ld.entities) if isinstance(e, IncludeLaunchDescription)
    ]
    matched = any(
        "nav2.launch.py" in str(inc.launch_description_source.location)
        for inc in includes
    )
    assert matched, "sim.launch.py must IncludeLaunchDescription(nav2.launch.py)"


# ----- package.xml ---------------------------------------------------------


def test_package_xml_declares_nav2_deps():
    tree = ET.parse(PACKAGE_XML)
    root = tree.getroot()
    deps = {e.text for e in root.findall("exec_depend")}
    deps |= {e.text for e in root.findall("depend")}
    for required in (
        "nav2_bringup",
        "nav2_msgs",
        "nav2_lifecycle_manager",
        "nav2_smac_planner",
        "nav2_controller",
        "nav2_planner",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_waypoint_follower",
    ):
        assert required in deps, f"package.xml missing exec_depend `{required}`"


# ----- setup.py installs nav2.yaml ----------------------------------------


def test_setup_py_installs_nav2_yaml():
    text = SETUP_PY.read_text()
    # config/*.yaml glob picks up nav2.yaml automatically; check the glob is in place.
    assert "config/*.yaml" in text, (
        "setup.py must install config/*.yaml (nav2.yaml lives there)"
    )
