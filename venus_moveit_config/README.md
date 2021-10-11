# venus_moveit_config
## Overview
This package contains the necessary config files to get Venus Manipulator working with MoveIt 2. Originally, the MoveIt Setup Assistant wizard was used to generate a MoveIt package for each robot individually. However, the MoveIt Setup Assistant has not been ported to ROS 2 so the generated launch files are manually migrated to ROS 2. 
The user are allowed to choose whether to have MoveIt work with physical robot hardware or a fake hardware.

## Usage
1. First load the controllers. The controllers are used to track the trajectory given by the planner:
```
ros2 launch venus_bringup venus_control.launch.py use_fake_hardware:=true start_rviz:=true
```
Make sure to set the `use_fake_hardware` arugment false if not using physical manipulator

2. Launch Rviz with Moveit 2:
```
ros2 launch venus_bringup venus_moveit.launch.py use_fake_hardware:=true
```