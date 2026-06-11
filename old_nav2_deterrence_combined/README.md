# Old Nav2 + Deterrence Combined

This folder contains an older combined test script from the Husky vineyard project.

## What this script does

`Full_System.py` combines two parts of the project:

1. **Nav2 waypoint navigation**
2. **Pest detection and deterrence**

The script sends Nav2 goals to the Husky. While the robot is navigating, it listens to the pest detection topic:

```text
/pest_detections
```

If a pest is detected, the script:

1. Cancels or pauses the current Nav2 goal.
2. Stops the robot.
3. Tries to turn the robot toward the detected pest.
4. Plays a predator sound.
5. Waits through a cooldown.
6. Resumes the navigation goal.

## Important note

This is not the final row-following code.

This was an earlier test that combined Nav2 waypoints with deterrence. The final navigation direction later moved toward LiDAR row following and mission-manager behavior.

Keep this file as a reference for future teams that want to combine pest detection/deterrence with navigation again.

## Main file

```text
Full_System.py
```

## Sound folder

The script expects predator sounds to be stored on the robot at:

```text
/home/administrator/Predator Sounds
```

The script maps pests to sound folders like:

```text
deer -> coyote
rodent -> owl or falcon
bird -> hawk or falcon
```
