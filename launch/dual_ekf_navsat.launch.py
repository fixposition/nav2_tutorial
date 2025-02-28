import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    rl_params_file = os.path.join(get_package_share_directory("nav2_tutorial"), 
                                  "config", "dual_ekf_navsat_params.yaml")

    return LaunchDescription([
        # Spawn EKF node for local odometry
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        
        # Spawn EKF node for global odometry
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        
        # Spawn Navsat transformation node
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[
                ("imu/data", "/fixposition/poiimu"),      # (Input) Message with orientation data
                ("odometry/filtered", "odometry/global"), # (Input) Robot's current position
                ("gps/fix", "fixposition/odometry_llh"),  # (Input) Robot's GPS coordinates
                ("odometry/gps", "odometry/gps"),         # (Output) Robot's GPS coordinates, transformed into its world frame
                ("gps/filtered", "gps/filtered"),         # (Output) Robot’s world frame position, transformed into GPS coordinates
            ],
        ),
    ])
