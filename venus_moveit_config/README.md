# venus_moveit_config
## Overview
This package contains the necessary config files to get Venus Manipulator worked with MoveIt 2. Originally, the MoveIt Setup Assistant wizard was used to generate a MoveIt package for each robot individually. However, the MoveIt Setup Assistant has not been ported to ROS 2 so the generated launch files are manually migrated to ROS 2. 
The user are allowed to choose whether to have MoveIt work with physical robot hardware or a fake hardware.

## Structure
```
venus_moveit_config/
├── config
├── rviz
└── srdf
```
### config
This folder contains config files writting in yaml format, including `controllers.yaml`, `kinematics.yaml` and `ompl_planning.yaml`. `controllers.yaml` tells Moveit which controller to use and define parameters in that controller. `kinematics.yaml` defines the kinematic solver used for kinematics and inverse kinematics calculation. `ompl_planning.yaml` determines planners and their parameters. Note that expect the default planner libraty OMPL, other libraries like CHOMP and STOMP can also be used. See https://ros-planning.github.io/moveit_tutorials/doc/chomp_planner/chomp_planner_tutorial.html (not officially ported to ros2 yet).

### rviz
This folder contains RViz config file that defines the window size, plugins to be include, initial viewing pose, etc.

### srdf
This folder contains SRDF files written using macro. SRDF is a format for representing semantic information about the robot structure. It does not replace URDF, and is not an extension of URDF. The SRDF refers to the joints and links in URDF and makes some definitions.

## Usage
<!-- 1. First load the controllers. The controllers are used to track the trajectory given by the planner:
```
ros2 launch venus_bringup venus_control.launch.py use_fake_hardware:=true start_rviz:=true
```
Make sure to set the `use_fake_hardware` arugment false if not using physical manipulator

2. Launch Rviz with Moveit 2:
```
ros2 launch venus_bringup venus_moveit.launch.py use_fake_hardware:=true
``` -->
See [venus_bringup](../venus_bringup/README.md)