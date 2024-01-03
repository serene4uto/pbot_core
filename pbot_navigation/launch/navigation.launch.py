import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    nav2param_file_name = 'navigation2_params.yaml'

    nav2param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('pbot_navigation'),
            'params',
            nav2param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    
    map_dir = LaunchConfiguration(
        'map',
        default='/workspaces/Project_Jackal_r2guide/pbot_ws/src/pbot_core/pbot_navigation/maps/home_room.yaml')
    
    # os.path.join(get_package_share_directory('pbot_navigation'), 'maps', 'home_room.yaml')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    
    return LaunchDescription([

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'autostart': autostart,
                'params_file': nav2param_dir,
                'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    ])