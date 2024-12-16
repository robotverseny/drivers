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
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import TimerAction, ExecuteProcess
from math import pi


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    lidar_dir = get_package_share_directory('lslidar_driver')
    usb_cam_launcher_dir = get_package_share_directory('usb_cam_launcher')
    launch_dir = os.path.join(bringup_dir, 'launch')
    lidar_luanch_dir = os.path.join(lidar_dir, 'launch')
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
    lidar_driver_start = LaunchConfiguration('lidar', default='true')
    lidar_driver_start_dec = DeclareLaunchArgument('lidar', default_value='true')
    cam_driver_start = LaunchConfiguration('camera', default='true')
    cam_driver_start_dec = DeclareLaunchArgument('camera', default_value='true')
    foxglove_start = LaunchConfiguration('foxglove', default='true')
    foxglove_start_dec = DeclareLaunchArgument('foxglove', default_value='true')
    joy_start = LaunchConfiguration('joy', default='false')
    joy_start_dec = DeclareLaunchArgument('joy', default_value='false')

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
                condition=IfCondition(lidar_driver_start),
    )

    cam_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(usb_cam_launcher_dir, 'launch', 'usb_cam_a.launch.py')),
                condition=IfCondition(cam_driver_start),
    )

    # Steering and path visualization in RViz with TimerAction to delay starting the node by X seconds (period)
    path_and_steer = TimerAction(
        period=3.0,
        actions=[
            launch_ros.actions.Node(
            package='turn_on_wheeltec_robot',
            executable='path_and_steering',
            output='screen',
            parameters=[
                {'publish_steer_marker': True}, 
                {'marker_topic': 'steer_marker'},
                {'marker_color': 'g'},
                {'map_frame': 'odom_combined'},
                {'marker_frame': 'base_link'},
                {'pose_frame': 'base_link'},
                {'cmd_topic': 'cmd_vel'},
                {'path_size': 500},
                # {'use_sim_time': False},
                ]
            )
        ]
    )

    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=[
                '--x',     '0.0',
                '--y',     '0.0',
                '--z',     '0.1',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0', #-
                '--frame-id',      'base_footprint',
                '--child-frame-id','base_link'
            ],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=[
                '--x',     '0.0',
                '--y',     '0.0',
                '--z',     '-0.1',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',      'base_footprint',
                '--child-frame-id','gyro_link'
            ],
    )

    map_to_odom_combined = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='map_to_odom_combined',
            arguments=[
                '--x',     '0.0',
                '--y',     '0.0',
                '--z',     '-0.1',
                '--roll',  '0.0',
                '--pitch', '0.0',
                '--yaw',   '0.0',
                '--frame-id',      'map',
                '--child-frame-id','odom_combined'
            ],
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
                'max_angular_vel': 0.8,
                'max_linear_vel': 0.5,
            }],
            output='screen',
            condition=IfCondition(joy_start),
    )

    joy_node = launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            parameters=[
                {'autorepeat_rate': 10.0},
            ], 
            condition=IfCondition(joy_start),
    )

    TimerAction (period = 2.0, actions=[ExecuteProcess(
        cmd=['ros2','topic','pub','--once', 'cmd_vel','geometry_msgs/msg/Twist',"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"],
        output='screen',)
        ],
        condition=IfCondition(joy_start),
    ),


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
            {'send_buffer_limit': 40000000 }, # 40MB avoiding occasional "Send buffer limit reached"
        ], 
        condition=IfCondition(foxglove_start),
    )


    ld = LaunchDescription()

    ld.add_action(carto_slam_dec)
    ld.add_action(lidar_driver_start_dec)
    ld.add_action(cam_driver_start_dec)
    ld.add_action(foxglove_start_dec)
    ld.add_action(joy_start_dec)
    ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(map_to_odom_combined)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(choose_car)
    ld.add_action(robot_ekf)
    ld.add_action(lslidar_driver)
    ld.add_action(cam_driver)
    ld.add_action(foxglove)
    ld.add_action(joy_translater_node) 
    ld.add_action(joy_node)
    ld.add_action(path_and_steer)


    return ld

