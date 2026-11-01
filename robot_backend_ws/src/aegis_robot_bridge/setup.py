import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aegis_robot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "deploy"), glob("deploy/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="robotics@example.com",
    description="USB-serial bridge between the dashboard and the ESP32 M3508 drive firmware.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_bridge_node = aegis_robot_bridge.node:main",
        ],
    },
)
