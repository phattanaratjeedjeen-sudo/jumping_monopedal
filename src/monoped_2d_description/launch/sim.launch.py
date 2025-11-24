import os
import time

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():

    package_name = "monoped_2d_description"
    controller_package_name = "monoped_controller"
    rviz_file_name = "config.rviz"

    bag_base_dir = os.path.join(os.path.expanduser("~"), "jumping_monopedal", "bag_files")
    os.makedirs(bag_base_dir, exist_ok=True)
    bag_output = os.path.join(bag_base_dir, f"bag_{int(time.time())}")


    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "1.0"

    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file_name)

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )


    # Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output],
            output='screen'
        )


    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity",
        arguments=[
            "-topic", "robot_description",
            "-entity", "monoped_2d",
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


    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_effort_controller',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
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

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[effort_spawner],
            )
        )
    )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=effort_spawner,
    #             on_exit=[bag],
    #         )
    #     )
    # )


    # Add launch actions
    launch_description.add_action(rviz)
    launch_description.add_action(world_arg)
    launch_description.add_action(gz_sim)
    launch_description.add_action(rsp)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(bridge)


    return launch_description
