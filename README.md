# Venus Manipulator ROS Packages
## Overview
This repository contains ROS packages meant to be used with Venus Manipulator. Packages were developed and tested on Ubuntu 20.04 using ROS 2 Foxy. Additionally, all ROS nodes were written using Python or C++. However, any programming language capable of sending ROS messages can be used to control the robots. To that effect, the core packages inside this repo are as follows:

- venus_bringup - contains combined launch files intergating all the functionalities
- venus_description - contains the meshes and URDFs
- venus_hardware - contains hardware interface files used to interact with actuators on joints
- venus_moveit_config - contains the configuration files necessary to launch an arm using MoveIt either on the physical robot, or just in Rviz
- venus_test_nodes - contains test files, can be viewed as examples or demos

## Requirements
- Computer running Ubuntu Linux 20.04 (note that virtual Linux machines have NOT been tested)
- A Venus Manipulator