"""HIL launch — same as vtol_sim.launch.py but interceptor runs on the RPi.

interceptor_node is replaced by interceptor_bridge_node, which forwards
odometry to the RPi and applies the RPi's computed pose to Gazebo.

Before launching, set your IPs (copy .env.example to .env first):
  export RPI_IP=<your-rpi-ip>

Terminal 1 (this launch file):
  ros2 launch vtol_sim vtol_sim_hil.launch.py

Terminal 2 (keyboard control):
  ros2 run vtol_sim keyboard_teleop

RPi (already running after deploy_rpi.sh --start):
  python3 ~/interceptor/interceptor_runner.py --pc-ip <your-pc-ip>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    rpi_ip = os.environ.get('RPI_IP', '')
    if not rpi_ip:
        raise RuntimeError(
            "RPI_IP environment variable is not set.\n"
            "Copy .env.example to .env, fill in your RPi IP, then:\n"
            "  export RPI_IP=<your-rpi-ip>\n"
            "  ros2 launch vtol_sim vtol_sim_hil.launch.py"
        )

    pkg = get_package_share_directory('vtol_sim')
    world = os.path.join(pkg, 'worlds', 'vtol_world.sdf')
    models_dir = os.path.join(pkg, 'models')

    gz_resource_path = models_dir
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        gz_resource_path = gz_resource_path + ':' + existing

    # Full Gazebo with the 3D GUI (as before).
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
            '/model/x3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        parameters=[{'config_file': os.path.join(pkg, 'config',
                                                  'gz_bridge_services.yaml')}],
        output='screen',
    )

    # HIL: bridge node talks to RPi instead of running guidance locally
    interceptor_bridge = Node(
        package='vtol_sim',
        executable='interceptor_bridge_node',
        name='interceptor_bridge_node',
        parameters=[{
            'rpi_ip':      rpi_ip,
            'guidance_law': 'apn',
        }],
        output='screen',
    )

    game_manager = Node(
        package='vtol_sim',
        executable='game_manager',
        name='game_manager',
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/X3/camera/image_raw'],
        output='screen',
    )

    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/X3/camera/image_raw'],
        output='screen',
    )

    rqt_minimap = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/game/minimap'],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        service_bridge,
        interceptor_bridge,
        game_manager,
        camera_bridge,
        rqt_image_view,
        rqt_minimap,
    ])
