import os
from ament_index_python.packages import get_package_share_directory

from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory("nav2_tutorial")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, 
        root_key="", 
        param_rewrites="", 
        convert_types=True
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ'
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz'
    )

    # Load robot model
    scout_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_description.launch.py'))
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
            #"log_level": "debug",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_mapviz'))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)

    # Add actions
    ld.add_action(scout_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)
    
    # Add the set_datum node
    datum_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="nav2_tutorial",
                executable="set_datum",
                name="set_datum",
                output="screen"
            )
        ]
    )
    ld.add_action(datum_node)

    return ld
