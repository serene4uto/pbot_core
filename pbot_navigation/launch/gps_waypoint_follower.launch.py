
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    nav2_params = os.path.join(
        get_package_share_directory("pbot_navigation"),
        "config", 
        "nav2_no_map_params.yaml")
    
    config_nav2_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')
    
    navigation_group_action = GroupAction([

        # robot_localization ekf
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('pbot_navigation'), 'launch', 'dual_ekf_navsat.launch.py']
                )
            )
        ),

        # navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
                )
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": config_nav2_params,
                "autostart": "True",
            }.items(),
        ),
    ])

    display_group_action = GroupAction([
        # rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'rviz_launch.py']
            )),
            condition=IfCondition(use_rviz),
        ),

        # mapviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pbot_navigation'), 'launch', 'mapviz.launch.py']
            )),
            condition=IfCondition(use_mapviz),
        ),
    ])


    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(navigation_group_action)
    ld.add_action(display_group_action)

    return ld