from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    output_final_position_arg = DeclareLaunchArgument(
        "output_final_position", default_value="false"
    )
    
    output_location_arg = DeclareLaunchArgument(
        "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
    )

    config_dual_ekf_navsat = PathJoinSubstitution(
        [FindPackageShare("pbot_navigation"), 
         "config", 
         "dual_ekf_navsat_params.yaml"],
    )

    node_ekf_filter_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[
            config_dual_ekf_navsat, 
            {"use_sim_time": False}
        ],
        remappings=[
            ("odometry/filtered", "odometry/local"),
        ],
    )

    node_ekf_filter_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            config_dual_ekf_navsat, 
            {"use_sim_time": False}
        ],
        remappings=[("odometry/filtered", "odometry/global")],
    )


    node_navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            config_dual_ekf_navsat, 
            {"use_sim_time": False}
        ],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(output_final_position_arg)
    ld.add_action(output_location_arg)
    ld.add_action(node_ekf_filter_odom)
    ld.add_action(node_ekf_filter_map)
    ld.add_action(node_navsat_transform)

    return ld