1.进入工作空间 
cd wheeltec_robot_ros2

2.编译lsn10功能包
colcon build --packages-select lsn10

3.source 工作空间所在的环境
source install/setup.bash

4.打开n10雷达节点
ros2 launch lsn10 ls_n10.launch.py
