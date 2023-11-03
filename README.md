# Robot Mission Control
Misson Control Node to handle multiple AMRs

## Action Messages

The message formats are present in the robot_action_interface package
along with all other interface definitions

## Action Server Logic

This Node runs on a server machine which has visibility of all robots which need to be controlled

### High Level Logic

The action servers are part of the state machine on the robot which will call on each
action server independently

![](images/High%20Level.png)


## Launching Action Servers

```bash
# With Namespacing (multi robot)
ros2 launch robot_mission_control mission_control.launch.py

# For Single Robot
ros2 launch robot_mission_control mission_control_single.launch.py

# Send example command
ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "robot_specific_dock_ids: [1, 2]"
```