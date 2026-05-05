"""Smoke test: OccupancyGrid conversion roundtrip."""
import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

from gpu_mi_ros2.mi_field_node import _occ_grid_to_numpy, _mi_to_occ_grid_msg


def _make_grid(W, H, res=0.5):
    msg = OccupancyGrid()
    msg.info = MapMetaData()
    msg.info.width = W
    msg.info.height = H
    msg.info.resolution = res
    msg.info.origin = Pose()
    data = np.random.randint(0, 100, W * H, dtype=np.int8)
    msg.data = data.tolist()
    return msg


def test_occ_roundtrip_shape():
    msg = _make_grid(20, 30)
    occ = _occ_grid_to_numpy(msg)
    assert occ.shape == (20, 30)
    assert occ.dtype == np.float32


def test_occ_values_in_range():
    msg = _make_grid(10, 10)
    occ = _occ_grid_to_numpy(msg)
    assert occ.min() >= 0.0
    assert occ.max() <= 1.0


def test_mi_to_grid_msg_shape():
    msg = _make_grid(15, 25)
    mi = np.random.rand(15, 25).astype(np.float32)
    out = _mi_to_occ_grid_msg(mi, msg)
    assert len(out.data) == 15 * 25


def test_unknown_cells_map_to_half():
    msg = _make_grid(4, 4)
    msg.data = [-1] * 16
    occ = _occ_grid_to_numpy(msg)
    assert np.allclose(occ, 0.5)
