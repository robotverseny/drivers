#!/bin/bash

# START


screen -ls | grep driver
screen -ls | grep lane



if ! screen -ls | grep -q "driver"; then
    echo -e "\e[42mStart driver\e[0m"
    screen -m -d -S driver bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py'
else
    echo -e "\e[41merror\e[0m driver already started"
fi
