import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import launch_ros.actions

def generate_launch_description():

    package_name = "monoped_description"
    rviz_file_name = "config.rviz"


    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.8"

    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file_name)
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )


    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 1 empty.sdf'}.items()
        # launch_arguments={'gz_args': '-v 1 empty.sdf'}.items()
    )

    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity",
        arguments=[
            "-topic", "robot_description",
            "-entity", "monoped",
            '-timeout', '120.0',
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
        ],
        output="screen"
    )


    # Controller Spawners
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

    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_effort_controller',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
    )

    spring_effort_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="monoped_description",
                executable="spring_effort_controller.py",
                name="spring_effort_controller",
                output="screen",
                parameters=[
                    {
                        "joint_name": "body_to_foot",
                        "spring_reference": 0.3,
                        "spring_stiffness": 5000.0,
                        "joint_damping": 50.0,
                    }
                ],
            )
        ],
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

    # Start RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_path],
        output="screen"
    )

    # Create LaunchDescription
    launch_description = LaunchDescription()

    # Start controllers in correct order
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner],
            )
        )
    )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=jsb_spawner,
    #             on_exit=[position_spawner],
    #         )
    #     )
    # )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[effort_spawner],
            )
        )
    )





    # Add launch actions
    launch_description.add_action(rviz)
    launch_description.add_action(gz_sim)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(bridge)
    launch_description.add_action(rsp)
    # launch_description.add_action(spring_effort_controller)


    return launch_description
