#!/usr/bin/env python3
'''
@company: Copyright (C) 2022, WHEELTEC (Dongguan) Co., Ltd
@product: LSn10
@filename: ls_n10.launch.py
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-12      Tues          ROS2

'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    frequency = LaunchConfiguration('frequency', default='5.5')
    serial_port_ = LaunchConfiguration('port', default='/dev/not_working_yet') # default port is /dev/wheeltec_laser
    baud_rate_ = LaunchConfiguration('baud_rate_', default='230400')
    frame_id_ = LaunchConfiguration('frame_id', default='laser')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    min_range = LaunchConfiguration('min_range', default='0.0')
    max_range = LaunchConfiguration('max_range', default='100.0')
    angle_disable_min = LaunchConfiguration('angle_disable_min', default='0.0')
    angle_disable_max = LaunchConfiguration('angle_disable_max', default='0.0')
    return LaunchDescription([

        DeclareLaunchArgument('serial_port_', default_value=serial_port_,
            description='Specifying port to connected lidar'),
            
        DeclareLaunchArgument('baud_rate_', default_value=baud_rate_,
            description='Specifying serial baudrate to connected lidar'),
            
        DeclareLaunchArgument('frame_id_', default_value=frame_id_,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

        DeclareLaunchArgument('scan_topic', default_value=scan_topic,
            description='Specifying scan_topic property of lidar'),

        DeclareLaunchArgument('min_range', default_value=min_range,
            description='Specifying min_range property of lidar'),

        DeclareLaunchArgument('max_range', default_value=max_range,
            description='Specifying max_range property of lidar'),

        DeclareLaunchArgument('angle_disable_min', default_value=angle_disable_min,
            description='Specifying angle_disable_min property of lidar'),

        DeclareLaunchArgument('angle_disable_max', default_value=angle_disable_max,
            description='Specifying angle_disable_max property of lidar'),

        launch_ros.actions.Node(
            package='lsn10',
            executable='lsn10',
            output='screen',
            parameters=[{'serial_port_': serial_port_, 
                         'baud_rate_': baud_rate_, 
                         'frame_id_': frame_id_,
                         'scan_topic': scan_topic, 
                         'min_range': min_range,
                         'max_range': max_range,
                         'angle_disable_min': angle_disable_min,
                         'angle_disable_max': angle_disable_max}],
             ),
    ])
