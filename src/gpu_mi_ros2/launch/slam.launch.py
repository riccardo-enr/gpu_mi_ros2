"""Launch slam_toolbox in online_async mode against the bridged /scan + TF.

Frames and scan topic must match the diff-drive plugin in
models/demo_robot/model.sdf and the ros_gz_bridge config.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("gpu_mi_ros2"), "config", "slam_toolbox.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from Gazebo",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params,
                description="Path to the slam_toolbox YAML params file",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    slam_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]
    )
