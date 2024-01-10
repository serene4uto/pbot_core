from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.conditions import IfCondition
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock')
    
    declare_2d_config_dir = DeclareLaunchArgument(
        name='cartographer_2d_config_dir',
        default_value= os.path.join(get_package_share_directory('pbot_mapping'), 'config'),
        description='Full path to 2d cartographer config directory'
    )

    declare_2d_config_basename = DeclareLaunchArgument(
        name='cartographer_2d_config_basename',
        default_value='2d_cartographer_cfg.lua',
        description='Name of 2d cartographer config file to load'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='False',
        description='Use rviz for visualization')
    
    declare_2d_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config',
        default_value=os.path.join(get_package_share_directory('pbot_mapping'), 'config', '2d_cartographer.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_enable_localization_cmd = DeclareLaunchArgument(
        name='enable_localization',
        default_value='True',
        description='Enable localization')
    
    declare_use_save_gui = DeclareLaunchArgument(
        name='use_save_gui',
        default_value='False',
        description='Enable save gui')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_save_gui = LaunchConfiguration('use_save_gui')

    enable_localization = LaunchConfiguration('enable_localization')

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('pbot_localization'), 'launch', 'ekf.launch.py']
            )
        ),
        condition=IfCondition(enable_localization),
    )

    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},],
        arguments=['-configuration_directory', LaunchConfiguration('cartographer_2d_config_dir'),
                   '-configuration_basename', LaunchConfiguration('cartographer_2d_config_basename')],
        remappings=[('scan', 'front/scan')]
    )

    node_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},],
        arguments=['-resolution', '0.05'],
        # remappings=[('map', 'pbot/scan_map')]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    node_save_gui = Node(
        package='pbot_mapping',
        executable='map_saver_gui',
        output='screen',
        condition=IfCondition(use_save_gui),
    )
    


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_2d_config_dir)
    ld.add_action(declare_2d_config_basename)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_2d_rviz_config_file)
    ld.add_action(declare_enable_localization_cmd)
    ld.add_action(declare_use_save_gui)
    ld.add_action(launch_localization)
    ld.add_action(node_cartographer)
    ld.add_action(node_occupancy_grid_node)
    ld.add_action(node_rviz)
    ld.add_action(node_save_gui)
    
    
    return ld