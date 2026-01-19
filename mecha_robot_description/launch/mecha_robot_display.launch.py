import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('mecha_robot_description')

    # Path to xacro file
    xacro_file = os.path.join(pkg_description, 'urdf', 'robots', 'custom_robot.urdf.xacro')
    robot_description_raw = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': False}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # This allows RViz to open with your saved settings
            arguments=['-d', os.path.join(pkg_description, 'rviz')]
        )
    ])