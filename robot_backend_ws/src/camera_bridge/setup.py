from setuptools import find_packages, setup

package_name = "camera_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/camera_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="robotics@example.com",
    description="ROS2 camera bridge that publishes dashboard image and health topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_bridge_node = camera_bridge.node:main",
        ],
    },
)