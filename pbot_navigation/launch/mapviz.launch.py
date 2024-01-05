
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("pbot_navigation")
    mapviz_config_file = os.path.join(gps_wpf_dir, "config", "gps_wpf_demo.mvc")


    # config_mapviz_mvc = PathJoinSubstitution(
    #     [FindPackageShare("pbot_navigation"), 
    #      "config", 
    #      "gps_wpf_demo.mvc"]
    # )

    node_mapviz = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[{"config": mapviz_config_file}]
    )

    node_initialize_origin = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "gps/fix"),
        ],
    )

    node_static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )


    ld = LaunchDescription()

    ld.add_action(node_mapviz)
    ld.add_action(node_initialize_origin)
    ld.add_action(node_static_transform_publisher)

    return ld
