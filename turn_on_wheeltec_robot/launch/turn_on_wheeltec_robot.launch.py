import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    lidar_dir = get_package_share_directory('lslidar_driver')
    launch_dir = os.path.join(bringup_dir, 'launch')
    lidar_luanch_dir = os.path.join(lidar_dir, 'launch')
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
            
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': 'true'}.items(),
    )
    #choose your car,the default car is mini_mec 
    choose_car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )
    # lidar driver (lslidar N10)
    lslidar_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(lidar_luanch_dir, 'lslidar_launch.py')),
    )
    
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

    robot_ekf = launch_ros.actions.Node(
            condition=UnlessCondition(carto_slam),
            package='robot_localization', 
            executable='ekf_node', 
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom_combined")]
            )
                              
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )

    joy_translater_node =launch_ros.actions.Node(
            #     condition=IfCondition(akmcar),
            package='turn_on_wheeltec_robot', 
            executable='joy_to_cmd', 
            parameters=[{
                'joy_topic': 'joy',
                'cmd_vel_topic': 'cmd_vel',
                'max_angular_vel': 0.22,
            }],
                output='screen'
            
    )

    joy_node = launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
    )

    foxglove = launch_ros.actions.Node(
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


    ld = LaunchDescription()

    ld.add_action(carto_slam_dec)
    ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(choose_car)
    ld.add_action(robot_ekf)
    ld.add_action(lslidar_driver)
    ld.add_action(foxglove)
    ld.add_action(joy_translater_node) ## TODO: fix ackermann cmd and cmd_vel issue
    ld.add_action(joy_node)

    return ld

