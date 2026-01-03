import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    # 1. Package Paths
    pkg_description = get_package_share_directory('mecha_robot_description')
    pkg_gazebo_world = get_package_share_directory('mecha_robot_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Environment (Models Path)
    install_dir = os.path.join(get_package_share_directory('mecha_robot_description'), '..', '..')
    gazebo_models_path = os.path.join(pkg_gazebo_world, 'models')

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(install_dir, 'share') + ':' + gazebo_models_path
    )

    # 3. Arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='pick_and_place_demo.world',
        description='Name of the Gazebo world file to load'
    )

    # 4. URDF
    urdf_file = os.path.join(pkg_description, 'urdf', 'robots', 'custom_robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    # 6. Gazebo Simulation
    world_path = PathJoinSubstitution([
        pkg_gazebo_world, 
        'worlds', 
        LaunchConfiguration('world_file')
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # 7. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'New_Robotic_Arm', '-z', '0.1'],
        output='screen'
    )

    # 8. Controllers Definition
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )

    # 9. DELAY CONTROLLERS (Crucial Fix)
    # This block waits for 'spawn_robot' to finish before starting the controllers
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster, arm_controller, gripper_controller],
        )
    )

    # 10. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_file_arg,
        bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delay_controllers  # Load the delayed block instead of nodes directly
    ])
