"""Structural tests for the RGBD camera on demo_robot (issue #8).

Verifies that the demo_robot SDF carries a single rgbd_camera sensor on a
camera_link, that ros_gz_bridge.yaml exposes the four ROS topics the issue
requires, and that sim.launch.py publishes the static base_link -> camera_link
transform at the SDF mount pose.
"""
import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution


PKG_DIR = Path(__file__).resolve().parents[1]
ROBOT_SDF = PKG_DIR / "models" / "demo_robot" / "model.sdf"
BRIDGE_YAML = PKG_DIR / "config" / "ros_gz_bridge.yaml"
SIM_LAUNCH = PKG_DIR / "launch" / "sim.launch.py"

# Mount pose chosen in the implementation plan: front of chassis, forward-facing.
EXPECTED_MOUNT_XYZ = (0.15, 0.0, 0.10)


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


# ----- SDF -----------------------------------------------------------------


def _model_root():
    tree = ET.parse(ROBOT_SDF)
    root = tree.getroot()
    # demo_robot SDF is wrapped in <sdf><model>... so locate the model element.
    model = root.find("model") if root.tag == "sdf" else root
    assert model is not None, "demo_robot SDF must contain a <model> element"
    return model


def test_sdf_has_camera_link_at_mount_pose():
    model = _model_root()
    cam_link = next(
        (link for link in model.findall("link") if link.get("name") == "camera_link"),
        None,
    )
    assert cam_link is not None, "demo_robot SDF must declare a <link name='camera_link'>"

    pose = cam_link.find("pose")
    assert pose is not None, "camera_link must have a <pose>"
    xyz = [float(v) for v in pose.text.split()[:3]]
    assert xyz == list(EXPECTED_MOUNT_XYZ), (
        f"camera_link pose xyz must be {EXPECTED_MOUNT_XYZ}, got {tuple(xyz)}"
    )


def test_sdf_has_camera_joint_to_base_link():
    model = _model_root()
    joints = model.findall("joint")
    cam_joints = [j for j in joints if "camera" in (j.get("name") or "")]
    assert cam_joints, "demo_robot SDF must declare a fixed camera joint"
    j = cam_joints[0]
    assert (j.get("type") or "fixed") == "fixed"
    parent = j.find("parent").text.strip()
    child = j.find("child").text.strip()
    assert parent == "base_link"
    assert child == "camera_link"


def test_sdf_has_rgbd_camera_sensor():
    model = _model_root()
    sensors = []
    for link in model.findall("link"):
        sensors.extend(link.findall("sensor"))
    rgbd = [s for s in sensors if s.get("type") == "rgbd_camera"]
    assert len(rgbd) == 1, (
        f"expected exactly one rgbd_camera sensor, got {len(rgbd)}"
    )
    sensor = rgbd[0]

    cam = sensor.find("camera")
    assert cam is not None, "rgbd_camera sensor must contain a <camera> block"

    image = cam.find("image")
    assert int(image.find("width").text) == 640
    assert int(image.find("height").text) == 480

    hfov = float(cam.find("horizontal_fov").text)
    assert 1.0 <= hfov <= 1.1, f"horizontal_fov ~1.047 rad expected, got {hfov}"

    rate = float(sensor.find("update_rate").text)
    assert rate >= 15.0, f"update_rate must be >= 15 Hz to satisfy AC, got {rate}"

    gz_frame = sensor.find("gz_frame_id")
    assert gz_frame is not None and gz_frame.text.strip() == "camera_link"


# ----- Bridge yaml ---------------------------------------------------------


def _bridge_entries():
    with BRIDGE_YAML.open() as f:
        return yaml.safe_load(f)


def test_bridge_exposes_color_image():
    entries = _bridge_entries()
    e = next((x for x in entries if x["ros_topic_name"] == "/camera/color/image_raw"), None)
    assert e is not None, "bridge must expose /camera/color/image_raw"
    assert e["ros_type_name"] == "sensor_msgs/msg/Image"
    assert e["direction"] == "GZ_TO_ROS"


def test_bridge_exposes_depth_pointcloud():
    entries = _bridge_entries()
    e = next((x for x in entries if x["ros_topic_name"] == "/camera/depth/points"), None)
    assert e is not None, "bridge must expose /camera/depth/points"
    assert e["ros_type_name"] == "sensor_msgs/msg/PointCloud2"
    assert e["direction"] == "GZ_TO_ROS"


def test_bridge_exposes_color_camera_info():
    entries = _bridge_entries()
    e = next(
        (x for x in entries if x["ros_topic_name"] == "/camera/color/camera_info"),
        None,
    )
    assert e is not None, "bridge must expose /camera/color/camera_info"
    assert e["ros_type_name"] == "sensor_msgs/msg/CameraInfo"


def test_bridge_exposes_depth_camera_info():
    entries = _bridge_entries()
    e = next(
        (x for x in entries if x["ros_topic_name"] == "/camera/depth/camera_info"),
        None,
    )
    assert e is not None, "bridge must expose /camera/depth/camera_info"
    assert e["ros_type_name"] == "sensor_msgs/msg/CameraInfo"


def test_bridge_camera_topics_target_camera_link_sensor():
    entries = _bridge_entries()
    cam_entries = [
        e for e in entries if e["ros_topic_name"].startswith("/camera/")
    ]
    assert cam_entries, "bridge must contain /camera/* entries"
    for e in cam_entries:
        gz = e["gz_topic_name"]
        assert "camera_link" in gz, (
            f"{e['ros_topic_name']} gz topic must reference camera_link, got {gz}"
        )


# ----- sim.launch.py -------------------------------------------------------


def test_sim_launch_publishes_camera_link_static_tf():
    mod = _load_launch_module(SIM_LAUNCH, "sim_launch_camera_tf")
    ld = mod.generate_launch_description()
    nodes = [e for e in _flatten(ld.entities) if isinstance(e, Node)]

    cam_tf = []
    for n in nodes:
        pkg = "".join(str(s) for s in (n.node_package or []))
        exe = "".join(str(s) for s in (n.node_executable or []))
        if pkg == "tf2_ros" and exe == "static_transform_publisher":
            text = _node_arg_text(n)
            if "camera_link" in text:
                cam_tf.append(text)

    assert cam_tf, (
        "sim.launch.py must include a static_transform_publisher with child-frame "
        "camera_link"
    )
    text = cam_tf[0]
    assert "base_link" in text
    # Mount pose components must appear as literals.
    for component in ("0.15", "0.10"):
        assert component in text, (
            f"static TF args must contain {component} for camera_link mount pose; "
            f"got: {text}"
        )
