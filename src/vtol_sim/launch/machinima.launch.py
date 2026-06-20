"""Launch the machinima ("Intercept") scene and record it.

  ros2 launch vtol_sim machinima.launch.py

Brings up Gazebo with the puppet-only machinima world, the service bridge the
director uses to drive poses, the image bridge that exposes the cine-cam feed to
ROS, and the director + recorder nodes. The director spawns the cast, plays the
shot list, and gates the recorder; the finished mp4 lands in ./media/.

Tip: for the cleanest footage run Gazebo's GUI maximized (or use the headless
`-s` server and rely solely on the recorded cine-cam feed).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vtol_sim')
    world = os.path.join(pkg, 'worlds', 'machinima_world.sdf')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
    )

    # Service bridge: director drives cine_cam + puppet poses via SetEntityPose.
    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        parameters=[{'config_file': os.path.join(
            pkg, 'config', 'gz_bridge_services_machinima.yaml')}],
        output='screen',
    )

    # Cine-cam image: Gazebo -> ROS sensor_msgs/Image for the recorder.
    cam_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/cine_cam/image'],
        output='screen',
    )

    director = Node(
        package='vtol_sim',
        executable='machinima_director',
        name='machinima_director',
        output='screen',
    )

    recorder = Node(
        package='vtol_sim',
        executable='machinima_recorder',
        name='machinima_recorder',
        parameters=[{'image_topic': '/cine_cam/image', 'fps': 30.0,
                     'out_dir': 'media'}],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        service_bridge,
        cam_bridge,
        director,
        recorder,
    ])
