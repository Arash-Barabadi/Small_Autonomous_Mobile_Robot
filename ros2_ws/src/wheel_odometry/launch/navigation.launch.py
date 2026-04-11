# launch/navigation.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg      = get_package_share_directory('wheel_odometry')
    nav2_dir = get_package_share_directory('nav2_bringup')

    nav2_config = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # ---------------------------------------------------------------
    # Map file argument — pass your map at launch time:
    # ros2 launch wheel_odometry navigation.launch.py map:=/path/to/map.yaml
    # ---------------------------------------------------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'my_map.yaml'),
        description='Full path to the map yaml file'
    )

    return LaunchDescription([

        map_arg,

        # ---------------------------------------------------------------
        # Base nodes (micro-ROS agent, TFs, odom, EKF)
        # ---------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot_base.launch.py')
            )
        ),

        # ---------------------------------------------------------------
        # Nav2 full stack
        # Includes: map_server, amcl, controller, planner,
        #           behavior_server, bt_navigator, lifecycle_manager
        # ---------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map':          LaunchConfiguration('map'),
                'params_file':  nav2_config,
                'use_sim_time': 'false',
            }.items(),
        ),

    ])

