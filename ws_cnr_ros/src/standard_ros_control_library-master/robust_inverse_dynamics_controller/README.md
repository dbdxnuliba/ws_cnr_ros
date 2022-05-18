# robust inverse dynamics control

required [rosdyn](https://github.com/JRL-CARI-CNR-UNIBS/robot_control_algorithms)

required parameters:
``` yaml
robust_inverse_dynamics:
  type: robot_control/RobustInverseDynamicsControl
  base_frame: world
  tool_frame: flange
  setpoint_topic_name: "/manipulator/joint_target"
  natural_frequency: 20   # natural frequency of the closed loop
  damping: 1              # relative damping of the closed loop
  robustness_gain: 0.01   # robustness gain

```
