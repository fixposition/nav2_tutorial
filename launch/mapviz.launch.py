import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

gps_wpf_dir = get_package_share_directory("nav2_tutorial")
mapviz_config_file = os.path.join(gps_wpf_dir, "config", "gps_wpf_demo.mvc")


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": mapviz_config_file}]
        ),
        
        # Spawn WGS84 to map transform
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "fixposition/odometry_llh"),
            ],
        ),
        
        # Set up static transform between map and origin
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='swri_transform',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'origin'
            ]
        )
    ])
