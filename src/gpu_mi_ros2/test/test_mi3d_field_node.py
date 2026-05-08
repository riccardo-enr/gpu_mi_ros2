"""Unit tests for mi3d_field_node and shared utils.

Requires `colcon build --packages-select octomap_to_grid gpu_mi_ros2` before
running, so that octomap_to_grid.msg.OccupancyGrid3D and the gpu_mi_ros2
package are importable.
"""

import numpy as np
import pytest
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

# These imports will fail until the implementation lands -- that is the red
# phase of TDD.
from octomap_to_grid.msg import OccupancyGrid3D
from gpu_mi_ros2 import utils


def _make_grid3d(W: int, H: int, D: int, res: float = 0.1,
                 origin=(0.0, 0.0, 0.0), fill: float = 0.5,
                 occupied=None) -> OccupancyGrid3D:
    msg = OccupancyGrid3D()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.origin = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
    msg.resolution = float(res)
    msg.width = W
    msg.height = H
    msg.depth = D
    data = np.full((W, H, D), fill, dtype=np.float32)
    if occupied is not None:
        for (i, j, k) in occupied:
            data[i, j, k] = 1.0
    # Flatten in (W, H, D) C order: index = i*H*D + j*D + k (k fastest).
    msg.data = data.reshape(-1).tolist()
    return msg


# -----------------------------------------------------------------------------
# utils.grid3d_msg_to_occ3d
# -----------------------------------------------------------------------------

def test_grid3d_msg_roundtrip_shape_and_dtype():
    msg = _make_grid3d(4, 5, 6, res=0.2, origin=(1.0, -2.0, 0.5))
    occ, origin, res = utils.grid3d_msg_to_occ3d(msg)
    assert occ.shape == (4, 5, 6)
    assert occ.dtype == np.float32
    assert occ.flags["C_CONTIGUOUS"]
    assert origin == (1.0, -2.0, 0.5)
    assert res == pytest.approx(0.2)


def test_grid3d_msg_preserves_voxel_indexing():
    msg = _make_grid3d(3, 4, 5, occupied=[(0, 0, 0), (2, 3, 4), (1, 2, 3)])
    occ, _, _ = utils.grid3d_msg_to_occ3d(msg)
    assert occ[0, 0, 0] == 1.0
    assert occ[2, 3, 4] == 1.0
    assert occ[1, 2, 3] == 1.0
    # Untouched voxels stay at the prior fill of 0.5.
    assert occ[1, 1, 1] == 0.5


def test_grid3d_msg_empty_grid():
    msg = _make_grid3d(0, 0, 0)
    occ, _, _ = utils.grid3d_msg_to_occ3d(msg)
    assert occ.shape == (0, 0, 0)


# -----------------------------------------------------------------------------
# utils.mi_field_to_pointcloud2
# -----------------------------------------------------------------------------

def test_pointcloud2_emits_only_above_threshold():
    mi = np.zeros((3, 3, 3), dtype=np.float32)
    mi[1, 1, 1] = 0.9
    mi[0, 0, 0] = 0.1
    pc = utils.mi_field_to_pointcloud2(
        mi, origin=(0.0, 0.0, 0.0), res=1.0, frame_id="map",
        min_mi=0.5, stamp=None,
    )
    assert isinstance(pc, PointCloud2)
    assert pc.header.frame_id == "map"
    # Two voxels above threshold? no -- only one.
    assert pc.width == 1
    assert pc.height == 1


def test_pointcloud2_has_xyz_intensity_fields():
    mi = np.ones((2, 2, 2), dtype=np.float32)
    pc = utils.mi_field_to_pointcloud2(
        mi, origin=(0.0, 0.0, 0.0), res=1.0, frame_id="map",
        min_mi=0.0, stamp=None,
    )
    field_names = [f.name for f in pc.fields]
    assert field_names == ["x", "y", "z", "intensity"]
    assert pc.width == 8  # all voxels emitted (min_mi=0 still excludes <=0.0? define inclusive)


def test_pointcloud2_xyz_at_voxel_centers():
    mi = np.zeros((2, 2, 2), dtype=np.float32)
    mi[1, 0, 1] = 2.0
    pc = utils.mi_field_to_pointcloud2(
        mi, origin=(10.0, 20.0, 30.0), res=0.5, frame_id="map",
        min_mi=0.5, stamp=None,
    )
    assert pc.width == 1
    # decode the single point (x, y, z, intensity) -- 4 float32 fields, 16 bytes
    assert pc.point_step == 16
    arr = np.frombuffer(bytes(pc.data), dtype=np.float32).reshape(-1, 4)
    assert arr.shape == (1, 4)
    x, y, z, intensity = arr[0]
    # Voxel (1, 0, 1) center -> origin + (i+0.5, j+0.5, k+0.5)*res
    assert x == pytest.approx(10.0 + 1.5 * 0.5)
    assert y == pytest.approx(20.0 + 0.5 * 0.5)
    assert z == pytest.approx(30.0 + 1.5 * 0.5)
    assert intensity == pytest.approx(2.0)


def test_pointcloud2_empty_when_all_below_threshold():
    mi = np.zeros((2, 2, 2), dtype=np.float32)
    pc = utils.mi_field_to_pointcloud2(
        mi, origin=(0.0, 0.0, 0.0), res=1.0, frame_id="map",
        min_mi=0.5, stamp=None,
    )
    assert pc.width == 0
    assert len(pc.data) == 0


# -----------------------------------------------------------------------------
# Mi3dFieldNode end-to-end (with stubbed gpu_mi_py)
# -----------------------------------------------------------------------------

def test_node_emits_non_empty_cloud_for_synthetic_grid(monkeypatch):
    import rclpy
    from gpu_mi_ros2 import mi3d_field_node as node_mod

    # Stub gpu_mi_py.compute to a deterministic non-zero output.
    def fake_compute(occ3d, origin, res, algo):
        out = np.zeros_like(occ3d)
        out[occ3d > 0.5] = 1.0  # MI = 1 wherever occupancy > 0.5
        return out

    monkeypatch.setattr(node_mod, "gpu_mi_py",
                        type("Stub", (), {"compute": staticmethod(fake_compute)}))
    monkeypatch.setattr(node_mod, "_GPU_AVAILABLE", True)

    rclpy.init()
    try:
        node = node_mod.Mi3dFieldNode()
        msg = _make_grid3d(4, 4, 4, res=0.1,
                           origin=(0.0, 0.0, 0.0),
                           occupied=[(1, 2, 3), (0, 0, 0)])

        published = []
        node._pub.publish = lambda m: published.append(m)

        node._compute_and_publish(msg)
        assert len(published) == 1
        out = published[0]
        assert isinstance(out, PointCloud2)
        assert out.header.frame_id == "map"
        assert out.width == 2  # two occupied voxels -> two MI=1 points
    finally:
        rclpy.shutdown()
