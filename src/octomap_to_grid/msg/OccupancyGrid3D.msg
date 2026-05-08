# Dense 3-D occupancy grid in [0, 1].
#
# Voxel (i, j, k) corresponds to world coordinates
#   x = origin.x + (i + 0.5) * resolution
#   y = origin.y + (j + 0.5) * resolution
#   z = origin.z + (k + 0.5) * resolution
#
# `data` is laid out with k (depth, z) fastest, j (height, y) next, i (width, x) slowest:
#   data[i * height * depth + j * depth + k]
# This matches a numpy array of shape (width, height, depth) reshaped from a
# C-contiguous flat buffer.
#
# Values:
#   1.0  -> occupied
#   0.0  -> free
#   prior (typically 0.5) -> unknown

std_msgs/Header header
geometry_msgs/Point origin
float32 resolution
uint32 width
uint32 height
uint32 depth
float32[] data
