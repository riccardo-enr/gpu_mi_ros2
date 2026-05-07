import os
import tempfile
from pathlib import Path

# Gazebo Harmonic does not auto-load system plugins; the upstream
# cyberzoo_office.sdf has none, so the lidar never produces scans without
# the Sensors plugin. Inject the standard plugin block at launch time so
# the submodule stays untouched.
_GZ_SYSTEM_PLUGINS = """\
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
"""


def _patched_world(src_path: str) -> str:
    """Return a temp SDF path = src_path with system plugins injected."""
    src = Path(src_path).read_text()
    if "gz::sim::systems::Sensors" in src:
        return src_path
    # Inject right after the opening <world ...> tag.
    import re
    patched = re.sub(
        r"(<world\b[^>]*>)",
        r"\1\n" + _GZ_SYSTEM_PLUGINS,
        src,
        count=1,
    )
    out = Path(tempfile.gettempdir()) / "gpu_mi_ros2_cyberzoo_office_patched.sdf"
    out.write_text(patched)
    return str(out)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")

    # parents[3] = repo root  (launch/ -> pkg/ -> src/ -> root); symlink resolves to source
    repo_root = Path(__file__).resolve().parents[3]
    world_sdf = _patched_world(
        str(repo_root / "external" / "PX4-gazebo-models" / "worlds" / "cyberzoo_office.sdf")
    )
    # model://cyberzoo is inside the models/ subdirectory of the submodule
    gz_models_path = str(repo_root / "external" / "PX4-gazebo-models" / "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = (
        f"{existing_resource_path}:{gz_models_path}" if existing_resource_path else gz_models_path
    )

    robot_sdf = PathJoinSubstitution([pkg_share, "models", "demo_robot", "model.sdf"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "ros_gz_bridge.yaml"])
    slam_launch_path = PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])
    mi_launch_path = PathJoinSubstitution([pkg_share, "launch", "mi_field.launch.py"])
    headless = LaunchConfiguration("headless")
    slam = LaunchConfiguration("slam")
    mi = LaunchConfiguration("mi")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server only, without the GUI",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description="Start slam_toolbox alongside the sim",
            ),
            DeclareLaunchArgument(
                "mi",
                default_value="true",
                description="Start mi_field_node alongside the sim",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world_sdf],
                additional_env={"GZ_SIM_RESOURCE_PATH": gz_resource_path},
                condition=UnlessCondition(headless),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-s", "-r", world_sdf],
                additional_env={"GZ_SIM_RESOURCE_PATH": gz_resource_path},
                condition=IfCondition(headless),
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",
                    "demo_robot",
                    "-file",
                    robot_sdf,
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0.15",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
                output="screen",
            ),
            # Delay so /scan + TF are flowing before slam_toolbox starts.
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch_path),
                        launch_arguments={"use_sim_time": "true"}.items(),
                        condition=IfCondition(slam),
                    ),
                ],
            ),
            # Delay so /map exists before mi_field_node subscribes.
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(mi_launch_path),
                        condition=IfCondition(mi),
                    ),
                ],
            ),
        ]
    )
