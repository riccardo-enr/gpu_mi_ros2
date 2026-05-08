"""OctoMap pipeline launch (issue #9).

Spawns:
- A static `map -> odom` identity transform: in sim DiffDrive odom is exact,
  so anchoring `map` to `odom` directly makes the chain `map -> odom ->
  base_link` geometrically correct without any custom node. A follow-up
  issue covers proper world-pose ground truth (via gz `/world/<w>/dynamic_pose/info`)
  for the UAV milestone where DiffDrive is replaced by noisy PX4 odom.
- octomap_server_node: builds a 3D OctoMap from /camera/depth/points.

Designed to be included from sim.launch.py (mode:=3d). Expects slam_toolbox
to be OFF; this launch claims the `map` frame.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")
    octomap_params = PathJoinSubstitution([pkg_share, "config", "octomap.yaml"])

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
                output="screen",
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                parameters=[octomap_params],
                remappings=[("cloud_in", "/camera/depth/points")],
                output="screen",
            ),
        ]
    )
