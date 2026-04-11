from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------------------------------------------------------
        # base_link -> base_laser  (LD14P LiDAR)
        # 4.125 cm above base, centred
        # ---------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '--x',     '0',
                '--y',     '0',
                '--z',     '0.04125',
                '--roll',  '0',
                '--pitch', '0',
                '--yaw',   '0',
                '--frame-id',       'base_link',
                '--child-frame-id', 'base_laser',
            ],
        ),

        # ---------------------------------------------------------------
        # base_link -> imu_link  (BNO085 IMU)
        # Slightly offset and rotated -90° yaw to align IMU axes
        # ---------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=[
                '--x',     '-0.0674',
                '--y',     '0.006355',
                '--z',     '-0.0053',
                '--roll',  '0',
                '--pitch', '0',
                '--yaw',   '-1.57',
                '--frame-id',       'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),

    ])
