"""Autonomous fixed-wing interception (v2).

Pure-Python, Gazebo-independent core:
  - fixed_wing.py    : 3D point-mass fixed-wing kinematics (coordinated turn)
  - guidance.py      : Pure Pursuit / True PN / Augmented PN guidance laws
  - engagement_sim.py: headless harness that validates guidance with no ROS/Gazebo

The ROS2 node (interceptor_node.py) wraps this core; the math here can be
unit-tested and tuned entirely offline, then deployed unchanged to embedded HW.
"""
