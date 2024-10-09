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
sudo apt install libpcap-dev libboost-all-dev ros-jazzy-nav2-msgs ros-jazzy-ackermann-msgs ros-jazzy-tf2-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-joint-state-publisher ros-jazzy-robot-localization ros-jazzy-usb-cam ros-jazzy-foxglove*
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
ros2 run lsn10 lsn10 --ros-args -r serial_port:=/dev/wheeltec_laser -r baud_rate:=230400
```
```r
ros2 launch lsn10 ls_n10.launch.py 
```
```r
sudo nano /etc/udev/rules.d/usb.rules
```
```r
KERNELS=="1-1.2:1.0", MODE:="0777", GROUP:="dialout",SYMLINK+="wheeltec_laser"
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


## Links
- [github.com/robotverseny/fyi](https://github.com/robotverseny/fyi)
