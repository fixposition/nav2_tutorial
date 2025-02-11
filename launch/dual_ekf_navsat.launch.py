import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("nav2_tutorial")
    rl_params_file = os.path.join(gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[
                    ("imu/data", "/fixposition/poiimu"),
                    ("gps/fix", "fixposition/odometry_llh"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            )
        ]
    )
