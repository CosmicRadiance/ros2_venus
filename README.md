# Venus Manipulator
## To-do
- [x] Visualize the manipulator using xacro macro
- [x] Implement controllers similar to ros2_control_demo package
- [ ] Transfer Moveit configuration to ROS 2
- [ ] Realize control the movement via actutaor SDK

## ROS Command for test
### Activate all the actuators
    rostopic pub -1 /INNFOS/enableActuator actuatorcontroller_ros/ActuatorArray "JointIDs:
    - 0"
If the power light flashes green, the actuators successfully start up.

### Change acutator mode to position profile
    rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs: [1,2,3,4,5,6]
    ActuatorMode: 4"

### Hardcoded positions for test
    rostopic pub /INNFOS/actuator_targets sensor_msgs/JointState "header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
    name: ['1','2','3','4','5','6']
    position: [0,0,0,0,0,0]
    velocity: [0,0,0,0,0,0]
    effort: [0,0,0,0,0,0]"

    rostopic pub /INNFOS/actuator_targets sensor_msgs/JointState "header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
    name: ['1','2','3','4','5','6']
    position: [0,-6,6,-10,0,0]
    velocity: [0,0,0,0,0,0]
    effort: [0,0,0,0,0,0]"

    rostopic pub /INNFOS/actuator_targets sensor_msgs/JointState "header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
    name: ['1','2','3','4','5','6']
    position: [3,-12,12,-15,0,0]
    velocity: [0,0,0,0,0,0]
    effort: [0,0,0,0,0,0]"