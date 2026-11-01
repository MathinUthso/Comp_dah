import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg= get_package_share_directory('aegis_description')
    xacro_file = os.path.join(pkg, 'urdf', 'aegis_bot.urdf.xacro')

    #process xacro into urdf string
    robot_description = xacro.process_file(xacro_file).toxml()

    #Robot state publisher - broadcasts TF frames from URDF
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':robot_description,
                'use_sim_time':False,
                }]
            )

    #GUI for joInt state pub
    joint_state_publisher_gui=Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            )

    #Rizzzzler
    rviz_config = os.path.join(pkg,'launch','aegis.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
     )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        ])
    
