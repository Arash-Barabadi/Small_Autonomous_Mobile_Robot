import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg      = get_package_share_directory('wheel_odometry')
    slam_dir = get_package_share_directory('slam_toolbox')

    slam_config = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([

        # Base nodes start immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot_base.launch.py')
            )
        ),

        # SLAM Toolbox starts after 5 seconds
        # giving TF tree time to fully establish
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_dir, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'params_file': slam_config,
                        'use_sim_time': 'false',
                    }.items(),
                )
            ]
        ),

    ])

