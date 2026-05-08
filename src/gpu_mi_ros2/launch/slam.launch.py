"""Launch slam_toolbox in online_async mode against the bridged /scan + TF.

slam_toolbox is a managed (lifecycle) node, so we have to drive it through
configure -> activate to get /map publishing. Pattern mirrors the upstream
slam_toolbox/launch/online_async_launch.py.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("gpu_mi_ros2"), "config", "slam_toolbox.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": False,
            },
        ],
        output="screen",
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[slam.launch] activating slam_toolbox"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
    )

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
            slam_node,
            configure_event,
            activate_event,
        ]
    )
