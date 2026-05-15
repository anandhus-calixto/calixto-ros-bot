import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="src/calixto-ros-bot/gazebo/maze.world",
        description="Path to the Gazebo world file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value="src/calixto-ros-bot/config/mapper_params_online_async.yaml",
        description="Path to the SLAM Toolbox params file",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("calixto-ros-bot"),
            "rviz",
            "lidar_transient_local.rviz",           
        ]),
        description="Full path to the RViz2 config file (.rviz)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 (set false for headless / CI runs)",
    )

    # ── Reusable substitutions ────────────────────────────────────────────────

    world            = LaunchConfiguration("world")
    use_sim_time     = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config      = LaunchConfiguration("rviz_config")
    rviz_enabled     = LaunchConfiguration("rviz")

    # ── 1. Gazebo Simulation ──────────────────────────────────────────────────
    #    Launched immediately at t=0

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("calixto-ros-bot"),
                "launch",
                "launch_sim.launch.py",
            ])
        ),
        launch_arguments={"world": world}.items(),
    )

    # ── 2. SLAM Toolbox ───────────────────────────────────────────────────────
    #    Delayed 5 s to allow Gazebo + robot state publisher to fully start

    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    ])
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time":     use_sim_time,
                }.items(),
            )
        ],
    )

    # ── 3. Nav2 Navigation Stack ──────────────────────────────────────────────
    #    Delayed 10 s to allow SLAM to initialise and publish /map

    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "navigation_launch.py",
                    ])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    # ── 4. RViz2 ─────────────────────────────────────────────────────────────
    #    Delayed 6 s (after SLAM starts) so TF tree is ready when RViz opens.
    #    Skipped entirely when rviz:=false.

    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz_enabled),
                output="screen",
            )
        ],
    )

    # ── Assemble ──────────────────────────────────────────────────────────────

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        slam_params_arg,
        rviz_config_arg,
        rviz_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])
