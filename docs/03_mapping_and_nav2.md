# Mapping and Nav2

This file explains the basic mapping and navigation process we used.

## Simple idea

Before you can run localization, you need a saved map.

The order is:

1. Create a map with SLAM.
2. Save the map.
3. Run localization using that saved map.
4. Run Nav2 on top of localization.
5. Send the robot goals.

## What we did

We first used the Clearpath tutorials to learn how to:

* create a map
* save a map
* localize on a saved map
* run Nav2
* send navigation goals in RViz

Later, we also tested Cartographer and used maps from the vineyard.

## Step 1: Start RViz

Open mobaxterm or other displ:

```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_1071
```

RViz lets you see the robot, LiDAR scan, map, costmap, and navigation goals.

## Step 2: Create a map with SLAM

Use the Clearpath SLAM or mapping launch file for the Husky.

The exact launch file may depend on the robot setup, so check the Clearpath demo package on the robot.

The idea is:

```bash
source /opt/ros/humble/setup.bash
ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=false
```

While SLAM is running drive the robot around the area you want to map.

## Step 3: Save the map

After the map looks good in RViz save it in maps folder


Save the map:

```bash
cd $HOME/maps
ros2 run nav2_map_server map_saver_cli -f MAPNAME \
  --ros-args -p map_subscribe_transient_local:=true -r 
__ns:=/a200_1071

```

This should create two files:

```text
my_map_name.yaml
my_map_name.pgm
```

The `.yaml` file stores the map settings.

The `.pgm` file stores the map image.

## Step 4: Run localization with the saved map

After the map is saved, launch localization:

```bash
source ~/clearpath/setup.bash

ros2 launch clearpath_nav2_demos localization.launch.py \
  setup_path:=$HOME/clearpath/ \
  use_sim_time:=false \
  map:=$HOME/maps/my_map_name.yaml
```

In RViz, use **2D Pose Estimate** to give the robot its starting position on the map  and point the arrow in the direction the robot is facing.

## Step 5: Run Nav2

Open a new terminal:

```bash
source ~/clearpath/setup.bash

ros2 launch clearpath_nav2_demos nav2.launch.py \
  setup_path:=$HOME/clearpath/ \
  use_sim_time:=false
```

Now you can send the robot a goal in RViz.

Use:

```text
Nav2 Goal
```

or:

```text
Navigate To Pose
```

## Recording Test Data

Record rosbag data during every test for the topics that you want to log.

Useful command:

```bash

ros2 bag record -o vineyard\_test\_01 \\

&#x20; /a200\_1071/sensors/lidar2d\_0/scan \\

&#x20; /a200\_1071/platform/odom/filtered \\

&#x20; /a200\_1071/vineyard\_row\_follow/status \\

&#x20; /a200\_1071/cmd\_vel

```


## Step 6: Test simple goals first

Do not start with the vineyard.

Test in this order:

1. Indoor room
2. Hallway
3. Outdoor open area
4. One vineyard row
5. Multiple vineyard rows

## What worked

Mapping and Nav2 worked well indoors because the robot could see walls, corners, and clear objects.

We were able to create maps, save maps, localize on those maps, and send navigation goals.

## What did not work well

The vineyard was harder.

The rows looked very similar to the 2D LiDAR. This caused row aliasing, where localization could sometimes jump between rows.

Because of this, static-map navigation was useful, but not enough by itself.

## Final takeaway

Use mapping and Nav2 to learn the system first.

But for vineyard row driving, the better final direction was LiDAR row following.

The saved map is still useful, but the row follower is better for staying centered inside a row.
