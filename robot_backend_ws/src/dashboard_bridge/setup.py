from setuptools import find_packages, setup


package_name = "dashboard_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics",
    maintainer_email="robotics@example.com",
    description="ROS2 bridge layer for the rescue robot dashboard backend.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dashboard_bridge_node = dashboard_bridge.nodes:main_dashboard_bridge",
            "camera_bridge_node = dashboard_bridge.nodes:main_camera_bridge",
            "thermal_bridge_node = dashboard_bridge.nodes:main_thermal_bridge",
            "hardware_control_node = dashboard_bridge.nodes:main_hardware_control",
            "flexbe_adapter_node = dashboard_bridge.nodes:main_flexbe_adapter",
            "diagnostics_node = dashboard_bridge.nodes:main_diagnostics",
            "emergency_control_node = dashboard_bridge.nodes:main_emergency_control",
        ],
    },
)
