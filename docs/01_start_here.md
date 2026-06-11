FILE: docs/01\_start\_here.md



\# Start Here



This repo is for the next team working on the Husky vineyard robot.



The goal is to help you understand what we did, how to run the code, and what still needs work.



\## What this project did



We used a Husky A200 robot to test vineyard navigation.



We tested:



\* ROS 2

\* RViz

\* SLAM mapping

\* AMCL localization

\* Nav2 navigation

\* LiDAR row following

\* GPS logging

\* Pest detection and deterrence



\## Biggest lesson



Indoor navigation worked much better than vineyard navigation.



The vineyard rows looked very similar to the LiDAR, so the robot could sometimes think it was in the wrong row.



Because of this, the best direction was LiDAR row following. The robot uses the LiDAR to stay centered between the left and right sides of the row.



\## Recommended order



Do this in order:



1\. Learn how to connect to the Husky through ethernet/SSH.

2\. Open RViz.

3\. Check that LiDAR data is working.

4\. Create a map using SLAM.

5\. Save the map as a `.yaml` and `.pgm` file.

6\. Run localization using the saved map.

7\. Run Nav2 on map.

8\. Test basic navigation indoors or in a hallway.

9\. Test the row follower in a hallway.

10\. Test one vineyard row.

11\. Only then test turning into the next row.



