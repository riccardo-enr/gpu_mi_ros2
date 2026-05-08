"""Nav2 bringup for demo_robot diff-drive (issue #11).

Spawns the five Nav2 servers required for greedy NBV (#12):
- planner_server (SmacPlanner2D over /projected_map)
- controller_server (DWBLocalPlanner with conservative caps)
- bt_navigator (default navigate_to_pose_w_replanning_and_recovery BT)
- behavior_server (spin / backup / drive_on_heading / wait)
- waypoint_follower

A single nav2_lifecycle_manager (autostart=true) drives them
unconfigured -> configured -> active. AMCL is intentionally NOT included --
in 3D mode the map -> odom anchor is a static identity TF and there is no
laser-based localization in this milestone.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2.yaml"])
    bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_to_pose_w_replanning_and_recovery.xml",
        ]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    common = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params, common],
                remappings=[("cmd_vel", "cmd_vel")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, common],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params, common],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_params,
                    common,
                    {"default_nav_to_pose_bt_xml": bt_xml},
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_params, common],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": _LIFECYCLE_NODES,
                    }
                ],
            ),
        ]
    )
