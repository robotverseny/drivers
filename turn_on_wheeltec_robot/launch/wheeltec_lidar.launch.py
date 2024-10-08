import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rplidar_dir = get_package_share_directory('rplidar_ros2')
    rplidar_launch_dir = os.path.join(rplidar_dir, 'launch')

    lidar_dir = get_package_share_directory('lidar_ros2')
    lidar_launch_dir = os.path.join(lidar_dir, 'launch')
    
    Lsm10_dir = get_package_share_directory('lsm10_v2')
    Lsm10_launch_dir = os.path.join(Lsm10_dir, 'launch')

    Lsn10_dir = get_package_share_directory('lsn10')
    Lsn10_launch_dir = os.path.join(Lsn10_dir, 'launch')
        
    Ld14_dir = get_package_share_directory('ldlidar_sl_ros2')
    Ld14_launch_dir = os.path.join(Ld14_dir, 'launch')
    
    Ld06_dir = get_package_share_directory('ldlidar_stl_ros2')
    Ld06_launch_dir = os.path.join(Ld14_dir, 'launch')
           
    rplidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar.launch.py')),)
            
    rplidar_A3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_a3_launch.py')),)
    rplidar_S1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_s1_launch.py')),)
    rplidar_S2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_s2_launch.py')),)
            
    Lsm10 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Lsm10_launch_dir, 'ls_m10.launch.py')),)
            
    Lsn10 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Lsn10_launch_dir, 'ls_n10.launch.py')),)
            
    Ld14 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Ld14_launch_dir, 'ld14.launch.py')),)
    Ld06 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Ld06_launch_dir, 'ld06.launch.py')),)
                  
    # Create the launch description and populate
    ld = LaunchDescription()
    #Select your radar here, options include:
    #rplidar_ros(A1、A2)、Lsm10、Lsn10、Ld14、Ld06
    ld.add_action(rplidar_ros)

    return ld

