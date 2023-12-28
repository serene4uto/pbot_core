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

        # Robot localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
                'robot_localization'), '/launch/ekf.launch.py'])
        ),

    ])