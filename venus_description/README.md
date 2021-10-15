# venus_descrption

## Overview
This package contains the URDFs and meshes for the Venus Manipulator. The STL files are located in the */meshes* directory. The URDFs for the robot are located in the */urdf* directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server. Note that all the other ROS packages in the sub-repo reference this package to launch the robot description.



## Structure
```
.
├── ActuatorController_SDK (not included here)
├── config
├── gazebo
├── launch
├── meshes
├── ros2_control
├── src
└── urdf
```
### ActuatorController_SDK
This folder contains the SDK of the actuator.

### config
This folder contains the config files of RViz, including plugins, window size and initial viewing pose.

### gazebo
This folder contains color config files

### launch
This folder contains launch file for visualization

### meshes
This folder contains STL files for each part of the manipulator

### ros2_control
This folder contains ros2_control config files. It also defines whether to use fake hardware or not. The joint config including the type of the interface and the limits are also defined.

### src
This folder contains a demo ros2 node. It allows the user to make the manipulator move by draging the slider on the GUI of joint state publisher.

### urdf
This folder contains URDFs of the manipulator.



## Usage
To run this package, type the line below in a terminal.
```
ros2 launch venus_description view_manipulator.launch.py
```
After that, the user should be able to see an RViz GUI with this manipulator in it and a joint state publisher GUI. It is posible to change the joint angle using slider in the joint state publisher GUI.
![](images/description_jsp_gui.png)

(Optional)
Run the following command to enable simultaneous moving both psysically and virtually:
```
ros2 run venus_description venus_control_node
```
Now, when the user drag the slider, the manipulator will also move physically.