import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable # <-- Added SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'calixto-ros-bot'

    # --- ADD THIS PART ---
    # This points Gazebo to your 'src' folder so it can find the 'calixto-ros-bot' directory
    # If you have built your workspace, you can also use the install share path.
    pkg_path = os.path.join(get_package_share_directory(package_name))
    parent_dir = os.path.dirname(pkg_path) # This gets the 'share' directory

    # Set the resource path so Gazebo finds the meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(parent_dir)]
    )
    # ----------------------

    # 1. Path to your bridge config file
    bridge_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'bridge_config.yaml'
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value="empty.sdf",
        description='World to load'
        )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn Entity
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'calixto-ros-bot',
                                   '-z', '0.01'],
                        output='screen')

    # 2. Add the Bridge Node
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path, # <-- Add this first
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
    ])
