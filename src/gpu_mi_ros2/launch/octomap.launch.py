"""OctoMap pipeline launch (issue #9).

Spawns:
- gt_pose_to_tf: republishes Gazebo PosePublisher's `<world>-><model>` as the
  residual `map -> odom` so the TF chain map -> odom -> base_link is anchored
  to the simulator's world.
- octomap_server_node: builds a 3D OctoMap from /camera/depth/points.

Designed to be included from sim.launch.py (mode:=3d) or run on top of an
already-running sim. Expects slam_toolbox to be OFF; this launch claims the
`map` frame.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")
    octomap_params = PathJoinSubstitution([pkg_share, "config", "octomap.yaml"])

    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="cyberzoo_office",
                description="Gazebo world name; also the parent frame in PosePublisher's TF stream",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="demo_robot",
                description="Gazebo model name; PosePublisher's child frame",
            ),
            Node(
                package="gpu_mi_ros2",
                executable="gt_pose_to_tf",
                name="gt_pose_to_tf",
                parameters=[
                    {
                        "world_frame_in": world_name,
                        "model_frame_in": model_name,
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "publish_rate": 30.0,
                        "use_sim_time": True,
                    }
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
