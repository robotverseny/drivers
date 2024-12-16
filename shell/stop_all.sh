#!/bin/bash

# publish STOP command (0 speed) 
ros2 topic pub --once -w 1 --max-wait-time-secs 0.5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep 0.2 # wait 0.2 sec
ros2 topic pub --once -w 1 --max-wait-time-secs 0.5 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# STOP
echo "[INFO] Killing Screens"
killall -9 screen
echo "[INFO] Wiping empty Screens"
screen -wipe

# list remaining nodes if any
ros2 node list
# list remaining screens if any
screen -ls

echo -e "\e[43;30m All drivers (screens) stopped. \e[0m"
