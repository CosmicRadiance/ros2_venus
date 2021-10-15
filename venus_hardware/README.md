# venus_hardware
## Overview
This package contains the hardware interface of the manipulator. A class `VenusHardwareInterface` is defined in this package. Inheirted from `BaseInterface`, it abstracts the physical hardware and the drivers (called hardware components) for the ros2_control framework (See http://control.ros.org/getting_started.html#resource-manager). 


## Structure
```
venus_hardware/
├── ActuatorController_SDK
├── include
└── src
```
### ActuatorController_SDK
This folder contains the driver of the actuator. The motion of the manipulator is realized by the rotation of each joint, i.e. the roation of each actuator. 

### include
This folder contains the header file of the hardware interface. Besides, a head file for C++ visibility support is included.  

### src
This folder contains the source file of the hardware interface. Several functions are defined here:
- configure: this function allocates space for the states (position and velocity here) and checks whether the number of state and interface matches that of joints. If everything is fine, the status will be set as CONFIGURED.
- export_state_interfaces: this function sets the state interfaces
- export_command_interfaces: this function sets the command interfaces
- start: as its name, this function starts the actuators, set their mode "Profiled Position" and set the position zero. If everything is fine, the status will be set as STARTED.
- stop: this function disables all actuators and set the status STOPPED.
- read: this function reads the current position and velocity from the actuator. They are then converted to joint positions and velocities by the steering ratio. 
- write: this functions sends the desired positions to actuators. 