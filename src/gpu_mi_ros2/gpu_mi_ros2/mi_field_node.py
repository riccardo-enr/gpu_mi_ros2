"""
MI Field Node.

Subscribes to a 2-D occupancy grid, computes the GPU mutual information
field via gpu_mi_py, and republishes it as a nav_msgs/OccupancyGrid on
~/mi_field.

Topics
------
  Subscriptions:
    ~/map  [nav_msgs/OccupancyGrid]  input occupancy grid

  Publications:
    ~/mi_field  [nav_msgs/OccupancyGrid]  MI field (float rescaled to [0,100])

Parameters
----------
  algo        : str   -- gpu_mi algorithm: fcmi | uniform_fsmi | approx_fsmi | fsmi
  publish_rate: float -- max publish rate in Hz (0 = publish every map update)
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

try:
    import gpu_mi_py

    _GPU_AVAILABLE = True
except ImportError:
    _GPU_AVAILABLE = False


def _occ_grid_to_numpy(msg: OccupancyGrid) -> np.ndarray:
    """Convert nav_msgs/OccupancyGrid to float32 array in [0, 1], shape (W, H)."""
    W, H = msg.info.width, msg.info.height
    raw = np.array(msg.data, dtype=np.int8).reshape(H, W)
    occ = np.where(raw < 0, 0.5, raw / 100.0).astype(np.float32)
    return occ.T  # (W, H), column-major matching gpu_mi convention


def _mi_to_occ_grid_msg(mi: np.ndarray, template: OccupancyGrid) -> OccupancyGrid:
    """Pack a float MI field into a nav_msgs/OccupancyGrid scaled to [0, 100]."""
    msg = OccupancyGrid()
    msg.header = template.header
    msg.info = template.info

    finite = mi[np.isfinite(mi)]
    mi_max = float(finite.max()) if finite.size > 0 else 1.0
    mi_max = mi_max if mi_max > 0.0 else 1.0

    scaled = np.clip(mi / mi_max * 100.0, 0.0, 100.0)
    scaled = scaled.T.flatten().astype(np.int8)  # back to (H, W) row-major
    msg.data = scaled.tolist()
    return msg


class MiFieldNode(Node):
    def __init__(self):
        super().__init__("mi_field_node")

        self.declare_parameter("algo", "fcmi")
        self.declare_parameter("publish_rate", 0.0)

        self._algo = self.get_parameter("algo").get_parameter_value().string_value
        rate_hz = self.get_parameter("publish_rate").get_parameter_value().double_value

        if not _GPU_AVAILABLE:
            self.get_logger().warn(
                "gpu_mi_py not found -- MI field will not be published. "
                "Install gpu_mi and run `pixi run install` in the gpu_mi repo."
            )

        self._pub = self.create_publisher(OccupancyGrid, "~/mi_field", 1)
        self._sub = self.create_subscription(OccupancyGrid, "~/map", self._map_cb, 1)

        self._timer = None
        if rate_hz > 0.0:
            self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)
        self._pending: OccupancyGrid | None = None

        self.get_logger().info(
            f"mi_field_node ready  algo={self._algo}  gpu={'yes' if _GPU_AVAILABLE else 'NO'}"
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        if self._timer is not None:
            self._pending = msg
        else:
            self._compute_and_publish(msg)

    def _timer_cb(self) -> None:
        if self._pending is not None:
            self._compute_and_publish(self._pending)
            self._pending = None

    def _compute_and_publish(self, msg: OccupancyGrid) -> None:
        if not _GPU_AVAILABLE:
            return

        occ = _occ_grid_to_numpy(msg)
        res = msg.info.resolution
        origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            0.0,
        )

        # gpu_mi_py.compute expects (W, H, D); use D=1 for 2-D maps
        occ3d = np.ascontiguousarray(occ[:, :, np.newaxis])

        try:
            mi3d = gpu_mi_py.compute(occ3d, origin, res, self._algo)
            mi = mi3d[:, :, 0]
        except Exception as exc:
            self.get_logger().error(f"gpu_mi_py.compute failed: {exc}")
            return

        self._pub.publish(_mi_to_occ_grid_msg(mi, msg))


def main(args=None):
    rclpy.init(args=args)
    node = MiFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
