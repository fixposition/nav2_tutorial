import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    rl_params_file = os.path.join(get_package_share_directory("nav2_tutorial"), 
                                  "config", "dual_ekf_navsat_params.yaml")

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/odom")],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/map")],
        ),
    ])
