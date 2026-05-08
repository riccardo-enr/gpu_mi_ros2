from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("gpu_mi_ros2"), "config", "params3d.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params_file),

        # Octomap -> dense 3-D grid
        Node(
            package="octomap_to_grid",
            executable="octomap_to_grid_node",
            name="octomap_to_grid_node",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[
                ("~/octomap_in", "/octomap_binary"),
                ("~/grid_out", "/octomap_grid"),
            ],
            output="screen",
        ),

        # 3-D MI field
        Node(
            package="gpu_mi_ros2",
            executable="mi3d_field_node",
            name="mi3d_field_node",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[
                ("~/grid", "/octomap_grid"),
                ("~/mi3d_field", "/mi3d_field"),
            ],
            output="screen",
        ),
    ])
