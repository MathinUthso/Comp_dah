"""Bring up the Aegis robot in Gazebo Classic and connect the dashboard adapter."""

from __future__ import annotations

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("aegis_description")
    sim_share = get_package_share_directory("aegis_sim")
    gazebo_share = get_package_share_directory("gazebo_ros")
    try:
        plugin_share = get_package_share_directory("aegis_gazebo_plugins")
        plugin_lib = os.path.join(os.path.dirname(plugin_share), "..", "lib")
        plugin_lib = os.path.abspath(plugin_lib)
        os.environ["GAZEBO_PLUGIN_PATH"] = plugin_lib + os.pathsep + os.environ.get("GAZEBO_PLUGIN_PATH", "")
    except Exception:
        pass

    xacro_path = os.path.join(description_share, "urdf", "aegis_bot.urdf.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()
    world_path = os.path.join(sim_share, "worlds", "rescue_arena.world")
    rviz_path = os.path.join(sim_share, "config", "sim.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    verbose = LaunchConfiguration("verbose")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "gui": gui,
            "verbose": verbose,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    spawn_model = Node(
        package="aegis_sim",
        executable="spawn_model",
        name="aegis_spawn_model",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "entity": "aegis_bot",
                "spawn_x": ParameterValue(LaunchConfiguration("spawn_x"), value_type=float),
                "spawn_y": ParameterValue(LaunchConfiguration("spawn_y"), value_type=float),
                "spawn_z": ParameterValue(LaunchConfiguration("spawn_z"), value_type=float),
                "spawn_yaw": ParameterValue(LaunchConfiguration("spawn_yaw"), value_type=float),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    adapter = Node(
        package="aegis_sim",
        executable="dashboard_adapter",
        name="aegis_dashboard_adapter",
        output="screen",
        parameters=[
            {
                "camera_fps": ParameterValue(LaunchConfiguration("camera_fps"), value_type=float),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start the Gazebo client."),
            DeclareLaunchArgument("rviz", default_value="false", description="Start RViz2."),
            DeclareLaunchArgument("world", default_value=world_path, description="Gazebo world file."),
            DeclareLaunchArgument("verbose", default_value="false", description="Verbose Gazebo output."),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("camera_fps", default_value="12.0"),
            gazebo,
            robot_state_publisher,
            spawn_model,
            adapter,
            rviz_node,
        ]
    )
