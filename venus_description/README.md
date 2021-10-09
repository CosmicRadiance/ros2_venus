# venus_descrption

## Overview
This package contains the URDFs and meshes for the Venus Manipulator. The STL files are located in the */meshes* directory. The URDFs for the robot are located in the */urdf* directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server. Note that all the other ROS packages in the sub-repo reference this package to launch the robot description.

## Usage
To make 
Download actuator's SDK
```
git clone https://github.com/innfos/innfos-cpp-sdk.git
```
then manually copy innfos-cpp-sdk/sdk to venus_description/ActuatorController_SDK
