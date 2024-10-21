#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )

    base_to_laser = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.5', '0', '0','0', '0','0','base_link','laser_link'],
    )

    dummy_publisher = launch_ros.actions.Node(
            package='lslidar_driver', 
            executable='dummy_lidar_pub', 
            output='screen',
    )

    foxglove_bridge = launch_ros.actions.Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
            {'tls': False},
            {'certfile': ''},
            {'keyfile': ''},
            #{'topic_whitelist': "'.*'"},
            {'max_qos_depth': 10},
            {'num_threads': 0},
            {'use_sim_time': False},
        ]
    )

    return LaunchDescription([
        base_to_link,
        base_to_gyro,
        base_to_laser,
        dummy_publisher,
        foxglove_bridge,
    ])