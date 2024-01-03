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
    
    return LaunchDescription([
       Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            parameters=[{
                'gain': 0.1,
                'zeta': 0.001,
                'publish_tf': False,
                'use_mag': False,
            }],
            # remappings=[
            #     ('/some/default/topic', topic_name),
            # ]
        ),
        
        # Robot localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
                'robot_localization'), '/launch/ekf.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

    ])