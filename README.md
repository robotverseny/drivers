# drivers
Wheeltec Roboworks drivers


ROS 2 packages  [![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/)

## SSH

IP can be from `192.168.101` to `192.168.104` look at [https://github.com/robotverseny/fyi](https://github.com/robotverseny/fyi?tab=readme-ov-file#robot-n%C3%A9v-%C3%A9s-sz%C3%ADn--robot-name-and-color)

```r
ssh wheeltec@192.168.0.101
```
password: `dongguan`

SSH no password: https://github.com/szenergy/szenergy-public-resources/wiki/H-SSH-no-password


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

`lslidar_driver` needs `libpcap0.8-dev`, not `libpcap-dev`.

```r
sudo apt install libpcap0.8-dev libpcl-dev libboost-all-dev ros-jazzy-nav2-msgs ros-jazzy-ackermann-msgs ros-jazzy-tf2-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-joint-state-publisher ros-jazzy-robot-localization ros-jazzy-usb-cam ros-jazzy-foxglove*
```
```r
cd ~/ros2_ws
```
```r
rosdep install --from-paths src --ignore-src -y
```
```r
rosdep update
```

### Build ROS 2 packages
```r
cd ~/ros2_ws
```
```r
colcon build --symlink-install --packages-select serial wheeltec_robot_msg lslidar_msgs lslidar_driver turn_on_wheeltec_robot wheeltec_robot_urdf usb_cam_launcher
```
`wheeltec_py_package` do not built with this command. (further work is needed TODO)

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

## Driver installation

```r
sudo dmesg | grep tty
```

```r
[    0.000000] Kernel command line: reboot=w coherent_pool=1M 8250.nr_uarts=1 pci=pcie_bus_safe snd_bcm2835.enable_compat_alsa=0 snd_bcm2835.enable_hdmi=1  smsc95xx.macaddr=2C:CF:67:33:1D:14 vc_mem.mem_base=0x3fc00000 vc_mem.mem_size=0x40000000  zswap.enabled=1 zswap.zpool=z3fold zswap.compressor=zstd multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc quiet splash
[    0.000198] printk: legacy console [tty1] enabled
[    0.017826] 107d001000.serial: ttyAMA10 at MMIO 0x107d001000 (irq = 15, base_baud = 0) is a PL011 rev2
[    0.625036] 107d50c000.serial: ttyS0 at MMIO 0x107d50c000 (irq = 39, base_baud = 6000000) is a Broadcom BCM7271 UART
[    0.625133] serial serial0: tty port ttyS0 registered
[    5.321466] usb 2-1.3: cp210x converter now attached to ttyUSB0
[    5.335873] usb 2-1.4: cp210x converter now attached to ttyUSB1
```

```r
lsusb 
```

```r
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 002: ID 1a40:0101 Terminus Technology Inc. Hub
Bus 002 Device 004: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 002 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 003 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 004 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 002: ID 05e3:0608 Genesys Logic, Inc. Hub
Bus 004 Device 006: ID 2bc5:050e Orbbec 3D Technology International, Inc USB 2.0 Camera
Bus 004 Device 007: ID 2bc5:060e Orbbec 3D Technology International, Inc ORBBEC Depth Sensor
Bus 005 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
```


```r
sudo nano /etc/udev/rules.d/usb.rules
```
```r
sudo su

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser"' >/etc/udev/rules.d/rplidar_laser.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser"' >/etc/udev/rules.d/ld14.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_wheeltec.rules

service udev reload
sleep 2
service udev restart

sleep 2
udevadm control --reload-rules
sleep 2
udevadm trigger
```
```r
ls /dev/wh*
```
Result:
```r
/dev/wheeltec_controller  /dev/wheeltec_laser
```

```r
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```r
cd ~/ros2_ws/src/drivers/turn_on_wheeltec_robot
```
```r
sudo chmod +x wheeltec_udev.sh
```
```r
./wheeltec_udev.sh
```

## Run the driver

Lidar driver:
```r
ros2 launch lslidar_driver lslidar_launch.py
```

Camera driver:
```r
ros2 launch usb_cam_launcher usb_cam_a.launch.py
```

All in one:
```r
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py lidar:=true camera:=true foxglove:=true joy:=false
```
You can select the `lidar`, `camera`, `foxglove`, `joy` with `true` or `false`.



## Copy robags (mcap)

```r
rsync -avzh --progress wheeltec@192.168.0.100:/home/wheeltec/bag/my_record1 /mnt/c/bag/
```

## Topic list

TODO: Update the list (this only the first version)

| Topic                       | Hz      | Msg Type                                      |
|-----------------------------|---------|-----------------------------------------------|
| `/ackermann_cmd`              | 0.06    | `ackermann_msgs/msg/AckermannDriveStamped`      |
| `/cmd_vel`                    | 0.06    | `geometry_msgs/msg/Twist`                       |
| `/scan`                       | 10.0    | `sensor_msgs/msg/LaserScan`                     |
| `/lslidar_point_cloud`        | 10.0    | `sensor_msgs/msg/PointCloud2`                   |
| `/robotvel`                   | 20.01   | `wheeltec_robot_msg/msg/Data`                   |
| `/robotpose`                  | 20.01   | `wheeltec_robot_msg/msg/Data`                   |
| `/imu`                        | 19.61   | `sensor_msgs/msg/Imu`                           |
| `/power_voltage`              | 1.63    | `std_msgs/msg/Float32`                          |
| `/odom_combined`              | 39.24   | `nav_msgs/msg/Odometry`                         |
| `/robot_description`          | N/A     | `std_msgs/msg/String`                           |
| `/tf`                         | 29.65   | `tf2_msgs/msg/TFMessage`                        |
| `/tf_static`                  | 0.10    | `tf2_msgs/msg/TFMessage`                        |
| `/joint_states`               | 10.00   | `sensor_msgs/msg/JointState`                    |


## Links
- [github.com/robotverseny/fyi](https://github.com/robotverseny/fyi)
