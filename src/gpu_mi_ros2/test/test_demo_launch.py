"""Structural tests for the unified demo launch (issue #7).

Verifies that the package ships:
- An rviz2 config (`config/demo.rviz`) with displays for /map, /mi_field,
  /scan, and TF, with `map` as the fixed frame.
- A `demo.launch.py` that includes `sim.launch.py`, spawns rviz2 with the
  demo config, and spawns teleop_twist_keyboard in a detected terminal
  emulator (ghostty preferred, with fallbacks).
- `package.xml` exec_depends on `rviz2` and `teleop_twist_keyboard`.
- `setup.py` installs `*.rviz` files into the package share.
- `pixi.toml` exposes `demo` and `teleop` tasks.
- `README.md` documents the demo workflow.
"""
import importlib.util
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def _cmd_text(cmd) -> str:
    """Concatenate the literal text of an ExecuteProcess.cmd list."""
    parts = []
    for token in cmd or []:
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


PKG_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_DIR.parents[1]
PACKAGE_XML = PKG_DIR / "package.xml"
SETUP_PY = PKG_DIR / "setup.py"
DEMO_LAUNCH = PKG_DIR / "launch" / "demo.launch.py"
DEMO_RVIZ = PKG_DIR / "config" / "demo.rviz"
PIXI_TOML = REPO_ROOT / "pixi.toml"
README = REPO_ROOT / "README.md"


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


def test_package_xml_declares_rviz2_and_teleop():
    tree = ET.parse(PACKAGE_XML)
    deps = {e.text for e in tree.getroot().findall("exec_depend")}
    deps |= {e.text for e in tree.getroot().findall("depend")}
    assert "rviz2" in deps, "package.xml must declare rviz2"
    assert "teleop_twist_keyboard" in deps, (
        "package.xml must declare teleop_twist_keyboard"
    )


def test_setup_py_installs_rviz_configs():
    text = SETUP_PY.read_text()
    assert "config/*.rviz" in text or '"*.rviz"' in text, (
        "setup.py must install *.rviz files into share/<pkg>/config"
    )


def test_demo_rviz_exists_and_has_required_displays():
    assert DEMO_RVIZ.is_file(), f"missing {DEMO_RVIZ}"
    with DEMO_RVIZ.open() as f:
        cfg = yaml.safe_load(f)

    vis = cfg["Visualization Manager"]
    assert vis["Global Options"]["Fixed Frame"] == "map"

    displays = vis["Displays"]
    classes = [d.get("Class", "") for d in displays]
    topics = []
    for d in displays:
        topic = d.get("Topic")
        if isinstance(topic, dict):
            topics.append(topic.get("Value", ""))
        elif isinstance(topic, str):
            topics.append(topic)

    assert any(c == "rviz_default_plugins/TF" for c in classes), "TF display required"
    assert any(c == "rviz_default_plugins/LaserScan" for c in classes), (
        "LaserScan display required"
    )

    map_displays = [d for d in displays if d.get("Class") == "rviz_default_plugins/Map"]
    map_topic_values = []
    for d in map_displays:
        t = d.get("Topic")
        map_topic_values.append(t.get("Value", "") if isinstance(t, dict) else t)
    assert "/map" in map_topic_values, "Map display for /map required"
    assert "/mi_field" in map_topic_values, "Map display for /mi_field required"

    assert "/scan" in topics, "LaserScan display must subscribe to /scan"


def test_demo_launch_module_importable():
    assert DEMO_LAUNCH.is_file(), f"missing {DEMO_LAUNCH}"
    mod = _load_launch_module(DEMO_LAUNCH, "demo_launch_under_test")
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_demo_launch_includes_sim_launch():
    mod = _load_launch_module(DEMO_LAUNCH, "demo_launch_includes_sim")
    ld = mod.generate_launch_description()
    includes = [
        e for e in _flatten(ld.entities) if isinstance(e, IncludeLaunchDescription)
    ]
    matched = any(
        "sim.launch.py" in str(inc.launch_description_source.location)
        for inc in includes
    )
    assert matched, "demo.launch.py must IncludeLaunchDescription(sim.launch.py)"


def test_demo_launch_spawns_rviz2_with_demo_config():
    mod = _load_launch_module(DEMO_LAUNCH, "demo_launch_rviz")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    rviz_nodes = []
    for n in nodes:
        pkg = "".join(str(s) for s in (n.node_package or []))
        exe = "".join(str(s) for s in (n.node_executable or []))
        if pkg == "rviz2" and exe == "rviz2":
            rviz_nodes.append(n)
    assert len(rviz_nodes) == 1, f"expected exactly one rviz2 Node, got {len(rviz_nodes)}"

    cmd_text = _cmd_text(rviz_nodes[0].cmd)
    assert "demo.rviz" in cmd_text, "rviz2 must be loaded with demo.rviz config"


def test_demo_launch_spawns_teleop_with_terminal_detection():
    mod = _load_launch_module(DEMO_LAUNCH, "demo_launch_teleop")
    ld = mod.generate_launch_description()
    # launch_ros.actions.Node subclasses ExecuteProcess, so filter Nodes out.
    procs = [
        e for e in _flatten(ld.entities)
        if isinstance(e, ExecuteProcess) and not isinstance(e, Node)
    ]

    teleop_procs = [p for p in procs if "teleop_twist_keyboard" in _cmd_text(p.cmd)]
    assert teleop_procs, "demo.launch.py must spawn teleop_twist_keyboard"

    cmd_text = _cmd_text(teleop_procs[0].cmd)
    assert "ghostty" in cmd_text, "teleop spawner must prefer ghostty"
    assert any(
        emu in cmd_text for emu in ("gnome-terminal", "konsole", "xterm")
    ), "teleop spawner must include a fallback terminal emulator"


def test_pixi_toml_has_demo_and_teleop_tasks():
    text = PIXI_TOML.read_text()
    assert re.search(r"^demo\s*=", text, re.MULTILINE) or "demo = {" in text or 'demo = "' in text, (
        "pixi.toml must define a `demo` task"
    )
    assert re.search(r"^teleop\s*=", text, re.MULTILINE) or "teleop = {" in text or 'teleop = "' in text, (
        "pixi.toml must define a `teleop` task"
    )
    assert "demo.launch.py" in text, "demo task must invoke demo.launch.py"
    assert "teleop_twist_keyboard" in text, "teleop task must invoke teleop_twist_keyboard"


def test_readme_documents_demo_workflow():
    text = README.read_text()
    assert re.search(r"^##\s+Demo", text, re.MULTILINE), (
        "README.md must contain a `## Demo` section"
    )
    assert "pixi run demo" in text, "README Demo section must mention `pixi run demo`"
    assert "pixi run teleop" in text, "README Demo section must mention `pixi run teleop`"
