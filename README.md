# gpu_mi_ros2

ROS 2 node that computes a GPU-accelerated mutual information field from a 2-D occupancy grid using [gpu_mi](https://github.com/riccardo-enr/gpu_mi).

## Prerequisites

- ROS 2 (Humble / Jazzy)
- CUDA-capable GPU

## Build

```bash
# Clone with submodules
git clone --recurse-submodules git@github.com:riccardo-enr/gpu_mi_ros2.git
cd gpu_mi_ros2

# Install gpu_mi Python bindings from submodule
pixi run install-gpu-mi

# Install ROS dependencies and build
pixi run deps
pixi run build
```

## Launch

```bash
pixi run launch
```

Remaps `/map` -> `mi_field_node/map` and publishes MI field on `/mi_field`.

## Parameters

| Parameter     | Default        | Description                                      |
|---------------|----------------|--------------------------------------------------|
| `algo`        | `fcmi`         | MI algorithm: `fcmi`, `uniform_fsmi`, `approx_fsmi`, `fsmi` |
| `publish_rate`| `0.0`          | Max publish rate Hz; 0 = every map update        |

## Topics

| Topic       | Type                     | Direction   |
|-------------|--------------------------|-------------|
| `~/map`     | `nav_msgs/OccupancyGrid` | Subscription |
| `~/mi_field`| `nav_msgs/OccupancyGrid` | Publication  |
