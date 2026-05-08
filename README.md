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

# If already cloned without --recurse-submodules, initialize submodules manually
git submodule update --init --recursive

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

## Demo

End-to-end Gazebo demo with the `cyberzoo_office` world, a diff-drive robot,
`slam_toolbox` building a live `/map`, `mi_field_node` publishing `/mi_field`,
and rviz2 visualising both layers.

```bash
pixi run demo
```

This single command starts:

- Gazebo Harmonic with `external/PX4-gazebo-models/worlds/cyberzoo_office.sdf`
- `ros_gz_bridge` (`/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`)
- `slam_toolbox` (online async, publishes `/map`)
- `mi_field_node` (publishes `/mi_field`)
- `rviz2` with `config/demo.rviz` (Map + MI Field + LaserScan + TF)
- `teleop_twist_keyboard` in a new terminal window (ghostty preferred, with
  fallback to gnome-terminal / konsole / xterm)

If no supported terminal emulator is installed, open a second terminal and run
the teleop manually:

```bash
pixi run teleop
```

Use `i / , / j / l` to drive the robot; the map and MI field update live.
