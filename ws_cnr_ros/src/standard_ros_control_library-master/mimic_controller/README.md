# mimic control

required [rosdyn](https://github.com/JRL-CARI-CNR-UNIBS/robot_control_algorithms)

required parameters:
``` yaml
mimic_controller:
  type: robot_control/MimicController
  joint_names: # controlled joints
  - jnt1
  - jnt2
  - jnt3

  leading_joint: "jnt1"  # leading joint

  # target position of a generic joint:  offset+multiplier*position of leading joint
  # target velocity of a generic joint:  multiplier*velocity of leading joint
  multiplier: [1.0, -1.0, 1.0] #  (default 1.0)
  offset: [0.0, 0.0, 0.0] # (default null)

  Kp: [1.0, 2.0, 3.0] # position gain of controller
  Kv: [1.0, 2.0, 3.0] # velocity gain of the controller (default null)
  Ki: [1.0, 2.0, 3.0] # integral gain of the controller (default null)
  max_effort: [50.0, 50.0, 50.0] # maximum effort of the joint
  setpoint_topic_name: "/manipulator/joint_target"

```


``` yaml
mimic_effort_controller:
  type: robot_control/MimicEffortController
  joint_names: # controlled joints
  - jnt1
  - jnt2
  - jnt3

  leading_joint: "jnt1"  # leading joint

  # target effort of a generic joint:  offset+multiplier*effort of leading joint

  multiplier: [1.0, -1.0, 1.0] #  (default 1.0)
  offset: [0.0, 0.0, 0.0] # (default null)

  setpoint_topic_name: "/manipulator/joint_target"

```
