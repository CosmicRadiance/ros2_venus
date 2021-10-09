# venus_descrption

## Overview
This package contains the URDFs and meshes for the Venus Manipulator. The STL files are located in the */meshes* directory. The URDFs for the robot are located in the */urdf* directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server. Note that all the other ROS packages in the sub-repo reference this package to launch the robot description.

## Usage
To run this package, type the line below in a terminal.
```
ros2 launch venus_description view_manipulator.launch.py
```
After that, the user should be able to see an RViz GUI with this manipulator in it and a joint state publisher GUI. It is posible to change the joint angle using slider in the joint state publisher.

(Optional)
Run the following command to enable simultaneous moving both psysically and virtually:
```
ros2 run venus_description venus_control_node
```
Now, when the user drag the slider, the manipulator will also move physically.