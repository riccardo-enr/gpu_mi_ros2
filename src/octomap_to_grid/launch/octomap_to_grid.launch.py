from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="octomap_to_grid",
            executable="octomap_to_grid_node",
            name="octomap_to_grid_node",
            parameters=[{"occ_prior": 0.5, "free_thresh": 0.5}],
            remappings=[
                ("~/octomap_in", "/octomap_binary"),
                ("~/grid_out", "/octomap_grid"),
            ],
            output="screen",
        ),
    ])
