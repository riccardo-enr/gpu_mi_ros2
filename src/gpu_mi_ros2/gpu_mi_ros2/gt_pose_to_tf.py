"""
Ground-truth pose to TF bridge.

Listens to /tf for the Gazebo PosePublisher's `<world> -> <model>` transform
and the DiffDrive plugin's `odom -> base_link` transform, then publishes the
residual `map -> odom = map_T_base * inv(odom_T_base)` so the rest of the
stack sees a consistent `map -> odom -> base_link` TF chain whose `map`
frame is anchored to the simulator's world.

In sim with a perfect DiffDrive odometry the residual is identity; the node
generalises to noisy odom (UAV / real hardware) without code changes.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def compute_map_to_odom(map_T_base: np.ndarray, odom_T_base: np.ndarray) -> np.ndarray:
    """Return the residual SE(3) map -> odom = map_T_base @ inv(odom_T_base)."""
    return map_T_base @ np.linalg.inv(odom_T_base)


def _transform_to_matrix(t) -> np.ndarray:
    """geometry_msgs.Transform -> 4x4 numpy SE(3)."""
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return M


def _matrix_to_transform_msg(M: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(M[0, 3])
    msg.transform.translation.y = float(M[1, 3])
    msg.transform.translation.z = float(M[2, 3])

    R = M[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    msg.transform.rotation.x = float(x)
    msg.transform.rotation.y = float(y)
    msg.transform.rotation.z = float(z)
    msg.transform.rotation.w = float(w)
    return msg


class GtPoseToTf(Node):
    def __init__(self):
        super().__init__("gt_pose_to_tf")

        self.declare_parameter("world_frame_in", "cyberzoo_office")
        self.declare_parameter("model_frame_in", "demo_robot")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 30.0)

        self._world_in = self.get_parameter("world_frame_in").value
        self._model_in = self.get_parameter("model_frame_in").value
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("publish_rate").value)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"gt_pose_to_tf  {self._world_in}->{self._model_in}  ==>  "
            f"{self._map_frame}->{self._odom_frame} (residual)"
        )

    def _tick(self) -> None:
        try:
            world_T_model = self._buffer.lookup_transform(
                self._world_in, self._model_in, Time()
            )
            odom_T_base = self._buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time()
            )
        except TransformException:
            return

        map_T_base = _transform_to_matrix(world_T_model.transform)
        odom_T_base_mat = _transform_to_matrix(odom_T_base.transform)
        map_T_odom = compute_map_to_odom(map_T_base, odom_T_base_mat)

        msg = _matrix_to_transform_msg(
            map_T_odom,
            self._map_frame,
            self._odom_frame,
            self.get_clock().now().to_msg(),
        )
        self._broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GtPoseToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
