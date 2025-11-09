#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package shares
    desc_pkg = get_package_share_directory('monoped_description')

    # Ensure GAZEBO_MODEL_PATH includes description meshes
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    # os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + os.path.join(desc_pkg, 'share')
    # else:
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(desc_pkg, 'share')

    # Robot description (xacro)
    xacro_file = PathJoinSubstitution([FindPackageShare('monoped_description'), 'urdf', 'robot_core.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    # controller_yaml = os.path.join(desc_pkg, 'config', 'controller.yaml')

    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 1 empty.sdf'}.items()
        # launch_arguments={'gz_args': '-v 1 empty.sdf'}.items()
    )

    # Robot state publisher
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create', 
        name='spawn_entity', 
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'soda',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0'
        ]
    )

    # Controller manager spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    position_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_position_controller',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ]
    )

    # RViz
    rviz_config = os.path.join(desc_pkg, 'rviz', 'config.rviz')
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz', 
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gz_sim,
        state_pub,
        spawn_entity,

        # After spawning the robot, start the joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),

        # After joint_state_broadcaster is active, start the effort controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[position_spawner]
            )
        ),
        bridge,
        rviz
    ])
