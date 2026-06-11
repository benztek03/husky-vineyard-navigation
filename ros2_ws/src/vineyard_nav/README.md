# vineyard_nav_starter

This is a starter ROS 2 Python package for the architecture discussed in chat:
- Nav2 handles headland / row-entry moves.
- A custom row follower handles motion inside a vine row.
- The mission manager alternates between the two.

## Important limits

This starter assumes:
1. You will still run your Clearpath platform and sensors services.
2. You will still run RViz, localization, and Nav2.
3. You will manually create good `entry_pose` and `exit_pose` poses in `config/mission.yaml`.
4. The row length is known approximately.
5. Your `cmd_vel` input is the user command input that goes through Clearpath's existing twist mux.

This is the fastest likely-to-work version. It is not a full research-grade row recognizer.

## Install in your workspace

```bash
mkdir -p ~/vineyard_ws/src
cp -r /path/to/vineyard_nav_starter ~/vineyard_ws/src/vineyard_nav
cd ~/vineyard_ws
colcon build --packages-select vineyard_nav
source install/setup.bash
```

## Run

Terminal 1:
```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_1071
```

Terminal 2:
```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_nav2_demos localization.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=false map:=$HOME/maps/a200_map_cart_map.yaml
```

Terminal 3:
```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=false
```

Terminal 4:
```bash
source ~/vineyard_ws/install/setup.bash
ros2 launch vineyard_nav vineyard_row_demo.launch.py mission_file:=/home/administrator/vineyard_ws/src/vineyard_nav/config/mission.yaml
```

## Tuning

Start with slow speeds:
- base_speed: 0.25 to 0.35
- max_speed: 0.40 to 0.50
- max_angular: 0.6 to 0.8

If it oscillates, reduce:
- center_gain
- heading_gain

If it does not react enough, increase them slowly.

If weeds still dominate, remap the field with the lidar mounted higher.
