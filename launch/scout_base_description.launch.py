from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Load robot model
    model_name = 'scout_vrtk.xacro'
    # model_path = os.path.join(get_package_share_directory('scout_description'), "urdf", model_name)
    # print(model_path)
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("agilex_demo"), "urdf", model_name]),
    ])

    # Launch robot state publisher
    return LaunchDescription([
        # Configure launcher
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='False',
            description='Use simulation clock if true'
        ),
        LogInfo(msg='use_sim_time: '),
        LogInfo(msg=LaunchConfiguration('use_sim_time')),
        
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
    ])
