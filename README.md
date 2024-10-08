# drivers
Wheeltec Roboworks drivers


ROS 2 packages  [![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
```r
cd ~/ros2_ws/src
```
```r
git clone https://github.com/robotverseny/drivers
```

### Required

```r
sudo apt update
```

```r
sudo apt install libboost-all-dev ros-jazzy-nav2-msgs ros-jazzy-ackermann-msgs ros-jazzy-tf2-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-joint-state-publisher ros-jazzy-robot-localization
```

### Build ROS 2 packages
```r
cd ~/ros2_ws
```
```r
colcon build --symlink-install --packages-select wheeltec_robot_msg lsn10 usb_cam turn_on_wheeltec_robot wheeltec_robot_urdf
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

## Add similar to `~/.bashrc`

```bash
source /opt/ros/jazzy/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1

my_ip=$(ip a | grep 192. | awk '{print $2}' | cut -d'/' -f1 | head -n 1)
echo -e "robo_f | \e[44m$my_ip\e[0m | fekete - black"
```
## Links
- [github.com/robotverseny/fyi](https://github.com/robotverseny/fyi)