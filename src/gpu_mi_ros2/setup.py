from setuptools import find_packages, setup
import os
from glob import glob

package_name = "gpu_mi_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "models", "demo_robot"),
         glob("models/demo_robot/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Riccardo Enrico",
    maintainer_email="riccardo.enrico97@proton.me",
    description="GPU-accelerated mutual information field node for ROS 2",
    license="MIT",
entry_points={
        "console_scripts": [
            "mi_field_node = gpu_mi_ros2.mi_field_node:main",
        ],
    },
)
