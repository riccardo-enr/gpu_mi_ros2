"""Runtime NavigateToPose probe (issue #11).

Skipped by default. Bring up `pixi run sim mode:=3d nav2:=true` in another
terminal first, then run:

    RUN_NAV2_RUNTIME=1 pytest src/gpu_mi_ros2/test/test_nav2_runtime.py

The test waits for the Nav2 stack to be active, then sends a NavigateToPose
goal to a free cell ~1 m in front of the robot's spawn pose and asserts the
action server reports SUCCEEDED within a generous timeout.
"""
import os

import pytest

pytestmark = [
    pytest.mark.runtime,
    pytest.mark.skipif(
        not os.environ.get("RUN_NAV2_RUNTIME"),
        reason="set RUN_NAV2_RUNTIME=1 with sim+nav2 already running to enable",
    ),
]


def test_navigate_to_pose_succeeds_to_free_cell():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose

    rclpy.init()
    node = Node("nav2_runtime_probe")
    try:
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=30.0), (
            "navigate_to_pose action server not available within 30 s -- "
            "is `pixi run sim mode:=3d nav2:=true` running?"
        )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        # demo_robot spawns at (0, 0, 0.15); 1 m forward is a free cell in cyberzoo.
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        handle = send_future.result()
        assert handle is not None and handle.accepted, "goal was not accepted"

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=120.0)
        outcome = result_future.result()
        assert outcome is not None, "no result within 120 s"
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, (
            f"NavigateToPose status {outcome.status}, expected SUCCEEDED"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
