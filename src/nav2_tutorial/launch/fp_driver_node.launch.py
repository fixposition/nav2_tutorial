import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare, LaunchConfiguration

def generate_launch_description():
    node_name  = LaunchConfiguration('node_name', default='fixposition_driver_ros2')
    config_arg = LaunchConfiguration('config',    default='fp_driver_config.yaml')
    launcher   = LaunchConfiguration('launcher',  default='')

    pkg_share = FindPackageShare('nav2_tutorial').find('nav2_tutorial')
    config_file = os.path.join(pkg_share, 'config', config_arg.perform({}))

    return LaunchDescription([
        Node(
          package='fixposition_driver_ros2',
          executable='fixposition_driver_ros2_exec',
          name=node_name,
          output='screen',
          respawn=True,
          respawn_delay=5,
          prefix=launcher,
          parameters=[config_file],
          arguments=['--ros-args', '--log-level', f'{node_name.get()}:=info']
        )
    ])
