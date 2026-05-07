from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")

    # parents[3] = repo root  (launch/ -> pkg/ -> src/ -> root); symlink resolves to source
    repo_root = Path(__file__).resolve().parents[3]
    world_sdf = str(
        repo_root / "external" / "PX4-gazebo-models" / "worlds" / "cyberzoo_office.sdf"
    )

    robot_sdf = PathJoinSubstitution([pkg_share, "models", "demo_robot", "model.sdf"])
    bridge_config = PathJoinSubstitution([pkg_share, "config", "ros_gz_bridge.yaml"])
    headless = LaunchConfiguration("headless")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server only, without the GUI",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world_sdf],
                condition=UnlessCondition(headless),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-s", "-r", world_sdf],
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
        ]
    )
