from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration, Command


def generate_launch_description():
    # Load robot model
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("scout_description"), "urdf/scout_v2/scout_v2.xacro"]),
    ])

    # Launch robot state publisher
    return LaunchDescription([
        # Configure launcher
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='False',
            description='Use simulation clock if true'
        ),
        LogInfo(msg=['use_sim_time: ', LaunchConfiguration('use_sim_time')]),
        
        # Invoke ROS node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }]
        ),
        
        # Launch static transform publisher from FP_POI to base_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_base_to_fp_poi',
        #     output='screen',
        #     arguments=[
        #         '--x', '0',
        #         '--y', '0',
        #         '--z', '0',
        #         '--yaw', '0',
        #         '--pitch', '0',
        #         '--roll', '0',
        #         '--frame-id', 'FP_POI',
        #         '--child-frame-id', 'base_link'
        #     ]
        # )
    ])
