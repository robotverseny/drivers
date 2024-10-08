import os
from pathlib import Path
import launch_ros.actions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,LogInfo,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
#aaaaaaaaaaaakm
    mini_akm = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_akm_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.125 ', '0', '0.1608','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.195', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    senior_akm = GroupAction([
    
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','senior_akm_robot.urdf')],),
            
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.26 ', '0', '0.228','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.34', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

    top_akm_bs = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_akm_bs_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.53 ', '0', '0.228','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.51', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

    top_akm_dl = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_akm_dl_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.497 ', '0', '0.228','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.58', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

#mmmmmmmmmmmmmmmec

    mini_mec = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_mec_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.048 ', '0', '0.18','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.195', '0', '0.25','3.14', '0','0','base_footprint','camera_link'],),
    ])

    senior_mec_no = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','senior_mec_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.1 ', '0', '0.165','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.18', '0', '0.3','0', '0','0','base_footprint','camera_link'],),
    ])

    senior_mec_dl = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','senior_mec_dl_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.165 ', '0', '0.235','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.255', '0', '0.35','0', '0','0','base_footprint','camera_link'],),
    ])

    top_mec_bs = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_mec_bs_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.155 ', '0', '0.195','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.24', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

    top_mec_dl = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_mec_dl_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.155 ', '0', '0.195','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.24', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

    senior_mec_EightDrive = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mec_EightDrive_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.207 ', '0', '0.228','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.32', '0', '0.2','0', '0','0','base_footprint','camera_link'],),
    ])

    top_mec_EightDrive = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mec_EightDrive_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.267  ', '0', '0.228','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.32', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])
#oooooooooooooooooomi

    mini_omni = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_omni_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.0  ', '0', '0.17','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.08', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    senior_omni = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','senior_omni_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.087  ', '0', '0.23','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.187', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

    top_omni = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_omni_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.149  ', '0', '0.23','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.25', '0', '0.32','0', '0','0','base_footprint','camera_link'],),
    ])

#dddddddddddddddddddiff

    mini_tank = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.02', '0', '0.155','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.14', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    mini_4wd = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.031', '0', '0.155','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.12', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    mini_diff = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.031', '0', '0.155','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.12', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    top_diff = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','top_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.087', '0', '0.195','3.1415', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.12', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    four_wheel_diff_bs = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','four_wheel_diff_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.157', '0', '0.385','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.08', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    four_wheel_diff_dl = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','four_wheel_diff_robot.urdf')],),
            
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.272', '0', '0.257','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.08', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])
    # Create the launch description and populate
    ld = LaunchDescription()

    #Select your car model here, the options are:
    #mini_akm, senior_akm, top_akm_bs, top_akm_dl, 
    #mini_mec, senior_mec_no, senior_mec_dl, top_mec_bs, top_mec_dl, senior_mec_EightDrive, top_mec_EightDrive, 
    #mini_omni, senior_omni, top_omni, 
    #mini_d_tank, mini_diff, wheeldiff , four_wheel_diff_dl
    ld.add_action(mini_akm)
    return ld

