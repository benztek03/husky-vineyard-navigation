# Husky Startup

This file explains how to connect to the Husky and start the basic ROS 2 navigation tools.

## 1. Connect to the same network

First, make sure your laptop and the Husky are on the same network.

This could be:

* lab WiFi
* Ethernet
* hotspot

We used the turtlewifi router and initially connected via ethernet to setup netplan file.
Hotpspot is okay for testing but may have connectivity issues so use router for big tests.

## 2. Find the Husky IP address

The Husky needs an IP address before you can SSH into it.

Example IP format:

```text
192.168.x.x
```

or

```text
172.20.x.x
```

From your laptop, test if you can reach the robot:

```bash
ping ROBOT_IP_ADDRESS
```

Example:

```bash
ping 192.168.0.37
```

If the ping works, try SSH.

## 3. SSH into the Husky

From your laptop terminal or Windows PowerShell:

```bash
ssh administrator@ROBOT_IP_ADDRESS
```

Example:

```bash
ssh administrator@192.168.0.37
```
Use the robot password which is default clearpath.

## 4. Netplan note

The Husky network settings are controlled by a netplan file.

To check the netplan files on the robot:

```bash
ls /etc/netplan/
```

To view the netplan file:

```bash
cat /etc/netplan/60-wireless.yaml
```

To edit it:

```bash
sudo nano /etc/netplan/60-wireless.yaml
```

After editing netplan, test it first:

```bash
sudo netplan try
```

Then apply it:

```bash
sudo netplan apply
```

Important: be careful editing netplan. A bad netplan file can make the robot lose network connection. 

```bash
sudo cp /etc/netplan/YOUR_NETPLAN_FILE.yaml /etc/netplan/YOUR_NETPLAN_FILE.yaml.bak
```

## 5. Check ROS topics

```bash
ros2 topic list
```

Important topics:

```text
/a200_1071/sensors/lidar2d_0/scan
/a200_1071/platform/odom/filtered
/a200_1071/cmd_vel
```

## 6. Launch RViz

Usually RViz is launched from the laptop or computer that has display access
One way to view RViz is to download MobaXterm
Session -> SSH -> remote host is ip adresss -> specify username check and put administrator

Then run this in MobaXTerm or display access device.

```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_1071
```

## 7. Create a map first if needed

Localization needs a saved map.
If you already have a saved map, use it.
If not, create a map first using SLAM, then save it as:
Follow 03_mapping_and_nav2 to find commands to make/save a map

```text
map_name.yaml
map_name.pgm
```

## 9. Launch localization with a saved map

Example:

```bash
source ~/clearpath/setup.bash

ros2 launch clearpath_nav2_demos localization.launch.py \
  setup_path:=$HOME/clearpath/ \
  use_sim_time:=false \
  map:=$HOME/maps/MAPNAME.yaml
```
replace MAPNAME

## 10. Launch Nav2

```bash
source ~/clearpath/setup.bash

ros2 launch clearpath_nav2_demos nav2.launch.py \
  setup_path:=$HOME/clearpath/ \
  use_sim_time:=false
```

## 11. Run row follower

```bash
source ~/clearpath/setup.bash
source ~/vineyard_ws/install/setup.bash
ros2 run vineyard_nav row_follower
```

## 12. Run mission manager

```bash
source ~/clearpath/setup.bash
source ~/vineyard_ws/install/setup.bash
ros2 run vineyard_nav mission_manager
```

## Basic startup order

Use this order:

1. Connect laptop and Husky to the same network.
2. Find or confirm the Husky IP address.
3. SSH into the Husky.
4. Source the Clearpath setup.
5. Check ROS topics.
6. Open RViz.
7. Create a map if needed.
8. Launch localization with a saved map.
9. Launch Nav2.
10. Run the row follower.
11. Run the mission manager.
