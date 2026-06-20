"""Machinima (in-engine cinematic) tools for the v2 interception sim.

Pure-math camera primitives live in `camera_moves`; the scenario (shot list)
lives in `shots`. Both are Gazebo-independent so they can be unit-tested
without ROS or a running simulator. The ROS2 nodes that drive Gazebo are
`vtol_sim.machinima_director` and `vtol_sim.machinima_recorder`.
"""
