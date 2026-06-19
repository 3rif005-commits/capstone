"""
Launch the VTOL simulation with camera visualization.

Terminal 1 (this launch file):
  ros2 launch vtol_sim vtol_sim.launch.py

Terminal 2 (keyboard control):
  ros2 run vtol_sim keyboard_teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vtol_sim')
    world = os.path.join(pkg, 'worlds', 'vtol_world.sdf')
    models_dir = os.path.join(pkg, 'models')

    # Gazebo needs to find our local x3_camera model
    gz_resource_path = models_dir
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        gz_resource_path = gz_resource_path + ':' + existing

    # Gazebo Sim
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen',
    )

    # ROS <-> Gazebo bridge
    # ] = ROS→Gz   [ = Gz→ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
            # NOTE: scoped by the *model* name, which is "x3" (lowercase) in
            # model.sdf -- not the "X3" robotNamespace used by the command
            # topics. Must match what OdometryPublisher actually advertises.
            '/model/x3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    # Camera image bridge (Gazebo → ROS2 sensor_msgs/Image)
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/X3/camera/image_raw'],
        output='screen',
    )

    # Camera viewer
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/X3/camera/image_raw'],
        output='screen',
    )

    # Bird's-eye mini-map (published by game_manager)
    rqt_minimap = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/game/minimap'],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        camera_bridge,
        rqt_image_view,
        rqt_minimap,
    ])
