from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("gpu_mi_ros2"), "config", "params.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params_file),
        Node(
            package="gpu_mi_ros2",
            executable="mi_field_node",
            name="mi_field_node",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[
                ("~/map", "/map"),
                ("~/mi_field", "/mi_field"),
            ],
            output="screen",
        ),
    ])
