"""
MI 3-D Field Node.

Subscribes to a dense 3-D occupancy grid (``octomap_to_grid/OccupancyGrid3D``,
typically produced from an OctoMap by ``octomap_to_grid_node``), runs
``gpu_mi_py.compute`` on the (W, H, D) occupancy array, and republishes the
3-D MI field as a ``sensor_msgs/PointCloud2`` carrying voxel-center XYZ and
``intensity = MI`` for every voxel above a configurable threshold.

Topics
------
  Subscriptions:
    ~/grid       [octomap_to_grid/OccupancyGrid3D]   input 3-D occupancy grid

  Publications:
    ~/mi3d_field [sensor_msgs/PointCloud2]           MI field point cloud

Parameters
----------
  algo         : str   -- gpu_mi algorithm (fcmi | uniform_fsmi | approx_fsmi | fsmi | csqmi | approx_csqmi)
  publish_rate : float -- max publish rate in Hz (0 = publish on every input)
  min_mi       : float -- only voxels with MI > min_mi are emitted as points
  occ_prior    : float -- (informational only here; the converter applies the prior)
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from octomap_to_grid.msg import OccupancyGrid3D
from sensor_msgs.msg import PointCloud2

from . import utils


try:
    import gpu_mi_py
    _GPU_AVAILABLE = True
except ImportError:
    gpu_mi_py = None  # type: ignore[assignment]
    _GPU_AVAILABLE = False


class Mi3dFieldNode(Node):
    def __init__(self):
        super().__init__("mi3d_field_node")

        self.declare_parameter("algo", "fcmi")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("min_mi", 0.0)
        self.declare_parameter("occ_prior", 0.5)

        self._algo = self.get_parameter("algo").get_parameter_value().string_value
        self._min_mi = float(self.get_parameter("min_mi").get_parameter_value().double_value)
        rate_hz = float(self.get_parameter("publish_rate").get_parameter_value().double_value)

        if not _GPU_AVAILABLE:
            self.get_logger().warn(
                "gpu_mi_py not found -- 3-D MI field will not be published. "
                "Run `pixi run install-gpu-mi`."
            )

        self._pub = self.create_publisher(PointCloud2, "~/mi3d_field", 1)
        self._sub = self.create_subscription(
            OccupancyGrid3D, "~/grid", self._grid_cb, 1
        )

        self._timer = None
        if rate_hz > 0.0:
            self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)
        self._pending: OccupancyGrid3D | None = None

        self.get_logger().info(
            f"mi3d_field_node ready  algo={self._algo}  min_mi={self._min_mi}  "
            f"gpu={'yes' if _GPU_AVAILABLE else 'NO'}"
        )

    def _grid_cb(self, msg: OccupancyGrid3D) -> None:
        if self._timer is not None:
            self._pending = msg
        else:
            self._compute_and_publish(msg)

    def _timer_cb(self) -> None:
        if self._pending is not None:
            self._compute_and_publish(self._pending)
            self._pending = None

    def _compute_and_publish(self, msg: OccupancyGrid3D) -> None:
        if not _GPU_AVAILABLE:
            return
        if msg.width == 0 or msg.height == 0 or msg.depth == 0:
            return

        occ3d, origin, res = utils.grid3d_msg_to_occ3d(msg)
        occ3d = np.ascontiguousarray(occ3d, dtype=np.float32)

        try:
            mi3d = gpu_mi_py.compute(occ3d, origin, res, self._algo)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"gpu_mi_py.compute failed: {exc}")
            return

        pc = utils.mi_field_to_pointcloud2(
            mi3d=np.asarray(mi3d, dtype=np.float32),
            origin=origin,
            res=res,
            frame_id=msg.header.frame_id or "map",
            min_mi=self._min_mi,
            stamp=msg.header.stamp,
        )
        self._pub.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    node = Mi3dFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
