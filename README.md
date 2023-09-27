# The robot_gui package

Author: Andres Alamo 
Date: September 2023

## Description **PENDING**

This package creates a graphical user interface similar to the below image.
The GUI has 4 sections that are:
1. Display received messages of robot's info
2. Control the robot movement by buttons
3. Tracking the current position of the robot
4. Request the travelled distance

![](robot_gui/proposed_layout/gui_example.png)

## Usage

- Clone to your ROS workspace 
- Compile and source: 
`cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Prepare the robot:
```
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics
```
- Make sure `roscore` is running  
- Execute: 
```  
rosrun robot_info agv_robot_info_node
rosrun distance_tracker_service distance_tracker_service
rosrun robot_gui robot_gui_node
``` 

## License
- BSD-3-Clause
- CVUI Library: Copyright (c) 2016 Fernando Bevilacqua. Licensed under the MIT license.

## Dependencies
- ROS Noetic
- OpenCV
- [CVUI](https://github.com/Dovyski/cvui)