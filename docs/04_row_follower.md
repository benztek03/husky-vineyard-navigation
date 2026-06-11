\# Row Follower and Mission Manager



This project has two main custom navigation scripts:



```text

row\_follower.py

mission\_manager.py

```



\## 1. Row Follower



The row follower drives the Husky straight down a row.



It uses the LiDAR to look at the left and right sides of the row and keeps the robot centered.



\### Main input



```text

/a200\_1071/sensors/lidar2d\_0/scan

```



\### Main output



```text

/a200\_1071/cmd\_vel

```



\### Status topic



```text

/a200\_1071/vineyard\_row\_follow/status

```



\### Run row follower



```bash

source \~/clearpath/setup.bash

source \~/vineyard\_ws/install/setup.bash

ros2 run vineyard\_nav row\_follower

```



\## 2. Mission Manager



The mission manager controls the higher-level behavior.



The row follower only focuses on staying centered in the row. The mission manager decides when the robot should:



\* start row following

\* stop row following

\* detect the end of a row

\* move forward at the end of a row

\* try a turn into the next row

\* continue the mission



\### Run mission manager



```bash

source \~/clearpath/setup.bash

source \~/vineyard\_ws/install/setup.bash

ros2 run vineyard\_nav mission\_manager

```



\## Testing order



Do not start in the vineyard first.



Test in this order:



1\. Run the row follower in a hallway.

2\. Confirm the robot can stay centered.

3\. Run the row follower in one vineyard row.

4\. Test stopping at the end of the row.

5\. Test the mission manager.

6\. Test turning into the next row.



\## Important note



The row follower worked best for driving straight inside a row.



The mission manager and row-to-row turning still need more work. Future teams should focus localizing at the end of the row and start of a row to make more reliable turns by using row markers.



