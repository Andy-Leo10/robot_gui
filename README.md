# The robot_gui package

Author: Andres Alamo 
Date: September 2023

## Description **PENDING**

This package creates a graphical user interface featuring five 
buttons to make the robot go left, right, forward, backwards or stop.
When a button a clicked, the GUI displays the current velocity
and publishes a message of type `Twist` to a predefined ROS topic.

## Usage **PENDING**

- Clone to your ROS workspace, for instance into `~/catkin_ws/src`  
- Compile and source: `cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Make sure `roscore` is running  
- Execute: `rosrun cmd_vel_publisher_gui cmd_vel_publisher_gui`  

![](docs/images/cmd_vel_publisher_gui_demo.gif)

## Manual testing **PENDING**

- Use the `rostopic echo` command to display the messages published by the teleoperation buttons:  
```
rostopic echo /cmd_vel
```

## License
- BSD-3-Clause
- CVUI Library: Copyright (c) 2016 Fernando Bevilacqua. Licensed under the MIT license.

## Dependencies
- ROS Noetic
- OpenCV
- [CVUI](https://github.com/Dovyski/cvui)