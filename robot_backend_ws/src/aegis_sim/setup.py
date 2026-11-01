import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aegis_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="robotics@example.com",
    description="Gazebo Classic simulation and dashboard adapter for the Aegis rescue robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard_adapter = aegis_sim.dashboard_adapter:main",
            "spawn_model = aegis_sim.spawn_model:main",
        ],
    },
)
