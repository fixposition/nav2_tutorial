import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    scout_pkg_share = os.path.join(get_package_share_directory('scout_base'), 'launch')
    driver_pkg_share = os.path.join(get_package_share_directory('nav2_tutorial'), 'launch')

    return LaunchDescription([
        # 1) Scout node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_pkg_share, 'scout_mini_base.launch.py')
            ),
        ),

        # 2) Driver node
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
               os.path.join(driver_pkg_share, 'fp_driver_node.launch')
            ),
        ),
    ])
