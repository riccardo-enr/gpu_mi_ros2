"""Unified demo launch (issue #7).

Brings up the full cyberzoo_office demo stack with a single command:
    pixi run demo

Wraps `sim.launch.py` (Gazebo + bridge + slam_toolbox + mi_field_node), adds
rviz2 with the `demo.rviz` config, and spawns teleop_twist_keyboard in a
detected terminal emulator (ghostty preferred, falling back to gnome-terminal,
konsole, then xterm). If no emulator is installed, the user can still run
`pixi run teleop` in a separate terminal.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_TELEOP_SPAWNER = r"""
TELEOP_CMD='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
for term in ghostty gnome-terminal konsole xterm; do
  if command -v "$term" >/dev/null 2>&1; then
    case "$term" in
      gnome-terminal) exec "$term" -- bash -lc "$TELEOP_CMD" ;;
      ghostty)        exec "$term" -e bash -lc "$TELEOP_CMD" ;;
      konsole)        exec "$term" -e bash -lc "$TELEOP_CMD" ;;
      xterm)          exec "$term" -e bash -lc "$TELEOP_CMD" ;;
    esac
  fi
done
echo "[demo] no terminal emulator found (ghostty/gnome-terminal/konsole/xterm)."
echo "[demo] open a new terminal and run: pixi run teleop"
"""


def generate_launch_description():
    pkg_share = FindPackageShare("gpu_mi_ros2")
    sim_launch_path = PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "demo.rviz"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch_path),
                launch_arguments={
                    "headless": "false",
                    "slam": "true",
                    "mi": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["bash", "-lc", _TELEOP_SPAWNER],
                output="screen",
            ),
        ]
    )
