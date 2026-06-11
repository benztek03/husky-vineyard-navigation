FILE: docs/05\_known\_issues.md



\# Known Issues



These are the main problems we ran into.



\## 1. ROS communication problems



Fast DDS gave us a ton of issues with LiDAR and ROS topics. **Cyclone DDS worked better!!!!!!**



\## 2. Vineyard rows look too similar



The robot could localize correctly indoors, but the vineyard rows looked very similar to the LiDAR. Sometimes AMCL thought the robot was in the wrong row.



This is why we moved toward LiDAR row following.



\## 3. LiDAR height matters



If the LiDAR is too low, it may see weeds or low branches. A future team should test a higher or adjustable LiDAR mount.



\## 4. Turning at the end of rows is hard



The robot could follow a row, but turning into the next row was harder because of wheel slip, odometry drift, and posts near the row ends.



\## 5. GPS was not accurate enough



Normal GPS was useful for rough logging, but not accurate enough to keep the robot centered in a vineyard row.



Future teams should try RTK-GPS if available.





