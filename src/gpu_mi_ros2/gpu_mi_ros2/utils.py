"""
Shared helpers for the MI field nodes.

Conversions between ROS messages and numpy arrays, and PointCloud2 packing
for the 3-D MI field. The 2-D OccupancyGrid helpers are shared with
`mi_field_node`; the 3-D OccupancyGrid3D helpers are used by
`mi3d_field_node`.

All occupancy arrays use float32 in [0, 1] with `0.5` for unknown cells, to
match the convention expected by `gpu_mi_py.compute`.
"""

from __future__ import annotations

import struct
from typing import Iterable, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# -----------------------------------------------------------------------------
# 2-D OccupancyGrid (nav_msgs)
# -----------------------------------------------------------------------------

def occgrid_to_occ2d(msg: OccupancyGrid) -> np.ndarray:
    """Convert ``nav_msgs/OccupancyGrid`` to a (W, H) float32 array in [0, 1].

    Unknown cells (value -1) map to 0.5. Known cells in [0, 100] map linearly
    to [0, 1].
    """
    W, H = msg.info.width, msg.info.height
    raw = np.array(msg.data, dtype=np.int8).reshape(H, W)
    occ = np.where(raw < 0, 0.5, raw / 100.0).astype(np.float32)
    return occ.T  # (W, H)


def mi_to_occgrid_msg(mi: np.ndarray, template: OccupancyGrid) -> OccupancyGrid:
    """Pack a float MI field into a ``nav_msgs/OccupancyGrid`` scaled to [0, 100]."""
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


# -----------------------------------------------------------------------------
# 3-D OccupancyGrid3D (octomap_to_grid)
# -----------------------------------------------------------------------------

def grid3d_msg_to_occ3d(msg) -> Tuple[np.ndarray, Tuple[float, float, float], float]:
    """Convert ``octomap_to_grid/OccupancyGrid3D`` to ``(occ3d, origin, res)``.

    ``occ3d`` has shape ``(W, H, D)`` C-contiguous float32 in [0, 1], indexed as
    ``occ3d[i, j, k]`` for the voxel centered at
    ``(origin.x + (i+0.5)*res, origin.y + (j+0.5)*res, origin.z + (k+0.5)*res)``.

    The msg ``data`` field is laid out with ``k`` fastest, then ``j``, then
    ``i`` (i.e. ``index = i*H*D + j*D + k``).
    """
    W, H, D = int(msg.width), int(msg.height), int(msg.depth)
    res = float(msg.resolution)
    origin = (
        float(msg.origin.x),
        float(msg.origin.y),
        float(msg.origin.z),
    )

    if W == 0 or H == 0 or D == 0:
        return np.zeros((W, H, D), dtype=np.float32), origin, res

    flat = np.asarray(msg.data, dtype=np.float32)
    occ = np.ascontiguousarray(flat.reshape((W, H, D)))
    return occ, origin, res


# -----------------------------------------------------------------------------
# PointCloud2 packing
# -----------------------------------------------------------------------------

def _xyzi_pointcloud2(points: np.ndarray, frame_id: str,
                      stamp=None) -> PointCloud2:
    """Build a ``sensor_msgs/PointCloud2`` from an (N, 4) float32 array (x, y, z, intensity)."""
    assert points.ndim == 2 and points.shape[1] == 4
    points = np.ascontiguousarray(points, dtype=np.float32)
    n = points.shape[0]

    header = Header()
    header.frame_id = frame_id
    if stamp is not None:
        header.stamp = stamp

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * n
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


def mi_field_to_pointcloud2(mi3d: np.ndarray,
                            origin: Tuple[float, float, float],
                            res: float,
                            frame_id: str,
                            min_mi: float = 0.0,
                            stamp=None) -> PointCloud2:
    """Emit a ``sensor_msgs/PointCloud2`` with one point per voxel where
    ``mi > min_mi``.

    Each point carries voxel-center XYZ in the given frame plus an
    ``intensity`` field equal to the MI value at that voxel.
    """
    if mi3d.size == 0:
        return _xyzi_pointcloud2(np.zeros((0, 4), dtype=np.float32), frame_id, stamp)

    mask = np.isfinite(mi3d) & (mi3d > min_mi)
    if not mask.any():
        return _xyzi_pointcloud2(np.zeros((0, 4), dtype=np.float32), frame_id, stamp)

    W, H, D = mi3d.shape
    ii, jj, kk = np.nonzero(mask)
    x = origin[0] + (ii + 0.5) * res
    y = origin[1] + (jj + 0.5) * res
    z = origin[2] + (kk + 0.5) * res
    intensity = mi3d[ii, jj, kk]

    pts = np.stack([x, y, z, intensity], axis=1).astype(np.float32, copy=False)
    return _xyzi_pointcloud2(pts, frame_id, stamp)
