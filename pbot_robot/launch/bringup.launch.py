from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get URDF via xacro
    robot_description_command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('pbot_description'), 'urdf', 'pbot.urdf.xacro']
            ),
            ' ',
        ]
    
    pbot_bringup_group_action = GroupAction([

        # Pbot Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('pbot_description'),
                     'launch',
                     'description.launch.py']
                )
            ),
            launch_arguments=[('robot_description_command', robot_description_command)]
        ),

        # Pbot Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pbot_control'), 'launch', 'control.launch.py']
            )),
            launch_arguments=[('robot_description_command', robot_description_command)]
        ),

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pbot_control'), 'launch', 'teleop_base.launch.py']
            ))
        ),

        # Joy Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pbot_control'), 'launch', 'teleop_joy.launch.py']
            ))
        ),

        # Diagnostics in Jackal Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_robot'), 'launch', 'diagnostics.launch.py']
            ))
        ),

        # Wireless Watcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
            )),
            launch_arguments=[('connected_topic', 'wifi_connected')]
        ),

        # MicroROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/jackal'],
            output='screen'),

        # Set ROS_DOMAIN_ID
        ExecuteProcess(
            cmd=[
                ['export ROS_DOMAIN_ID=0;'],
                [FindExecutable(name='ros2'),
                 ' service call /set_domain_id ',
                 ' jackal_msgs/srv/SetDomainId ',
                 '"domain_id: ',
                 EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                 '"']
            ],
            shell=True,
        ),


        # Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pbot_robot'), 'launch', 'sensors.launch.py']
            ))
        ),

    ])

    ld = LaunchDescription()
    ld.add_action(pbot_bringup_group_action)
    return ld