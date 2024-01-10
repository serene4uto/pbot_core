import yaml

from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    lc = LaunchContext()
    ld = LaunchDescription()

    # # Primary Lidar Environment Variables
    # primary_lidar_enable = EnvironmentVariable('PBOT_LASER', default_value='false')
    # primary_lidar_model = EnvironmentVariable('PBOT_LASER_MODEL', default_value='vlp16')
    # primary_lidar_ip = EnvironmentVariable('PBOT_LASER_HOST', default_value='192.168.8.201')
    # primary_lidar_topic = EnvironmentVariable('PBOT_LASER_TOPIC', default_value='front/scan')
    # primary_lidar_mount = EnvironmentVariable('PBOT_LASER_MOUNT', default_value='front_laser')

    # if (primary_lidar_enable.perform(lc)) == 'true':
    #     if (primary_lidar_model.perform(lc) == 'vlp16'):


    # # Primary 3D Lidar Environment Variables
    # primary_lidar_3d_enable = EnvironmentVariable('PBOT_3D_LASER', default_value='false')
    # primary_lidar_3d_model = EnvironmentVariable('PBOT_3D_LASER_MODEL', default_value='vlp16')
    # primary_lidar_3d_ip = EnvironmentVariable('PBOT_3D_LASER_HOST', default_value='192.168.8.201')
    # primary_lidar_3d_topic = EnvironmentVariable('PBOT_3D_LASER_TOPIC', default_value='front/points')
    # primary_lidar_3d_mount = EnvironmentVariable('PBOT_3D_LASER_MOUNT', default_value='front_velodyne')

    # if (primary_lidar_3d_enable.perform(lc)) == 'true':
    #     if (primary_lidar_3d_model.perform(lc) == 'vlp16'):
                
    config_velodyne_driver_vlp16 = PathJoinSubstitution(
        [FindPackageShare("velodyne_driver"),
        "config",
        "VLP16-velodyne_driver_node-params.yaml"],
    )

    config_velodyne_pointcloud_vlp16 = PathJoinSubstitution(
        [FindPackageShare("velodyne_pointcloud"),
        "config",
        "VLP16-velodyne_transform_node-params.yaml"],
    )

    config_velodyne_pointcloud_vlp16_calibration = PathJoinSubstitution(
        [FindPackageShare("velodyne_pointcloud"),
        "params",
        "VLP16db.yaml"],
    )

    config_velodyne_laserscan_vlp16 = PathJoinSubstitution(
        [FindPackageShare("velodyne_laserscan"),
        "config",
        "default-velodyne_laserscan_node-params.yaml"],
    )

    with open(config_velodyne_pointcloud_vlp16.perform(lc), 'r') as f:
        config_velodyne_pointcloud_vlp16_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    config_velodyne_pointcloud_vlp16_params['calibration'] = config_velodyne_pointcloud_vlp16_calibration
 
    node_velodyne_driver = Node(
        package='velodyne_driver', 
        executable='velodyne_driver_node', 
        name='velodyne_driver_node',
        output='both',
        # remappings={('scan', primary_lidar_3d_topic)},
        parameters=[
            config_velodyne_driver_vlp16,
            {'device_ip': '192.168.8.201'},
            # {'frame_id': 'front_velodyne'}
        ]
    )

    # Pointcloud
    node_velodyne_transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        remappings={('velodyne_points', 'front/points')},
        parameters=[
            config_velodyne_pointcloud_vlp16_params,
            # {'frame_id': 'front_points'}
        ])
    
    # Laser
    node_velodyne_laserscan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        remappings={
            ('scan', 'front/scan'), 
            ('velodyne_points', 'front/points')},
        parameters=[
            config_velodyne_laserscan_vlp16,
            # {'frame_id': 'front_scan'},
        ]
    )

    # Primary IMU
    #TODO: Add IMU
    node_um7_imu = Node(
        package='umx_driver',
        executable='um7_driver',
        output='both',
        parameters=[
            {'port': '/dev/ttyIMU'},
            {'baud': 115200},
            {'frame_id': 'imu_link'},
        ],
        remappings=[
            ('/imu/data', '/um7_imu/data'),
            ('/imu/mag', '/um7_imu/mag'),
            ('/imu/rpy', '/um7_imu/rpy'),
            ('/imu/temperature', '/um7_imu/temperature')
        ]
    )

    # GPS
    config_ublox_gps_zedf9p = PathJoinSubstitution(
        [FindPackageShare("ublox_gps"),
        "config",
        "zed_f9p.yaml"],
    )

    node_ublox_gps = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[
            config_ublox_gps_zedf9p,
        ],
        remappings=[
            ('/ublox_gps_node/fix', 'gps/fix'),
        ]
    )

    node_rtcm_provider = Node(
        package='rtcm_provider',
        executable='rtcm_ntrip_pub',
        output='both',
    )

    ld.add_action(node_velodyne_driver)
    ld.add_action(node_velodyne_transform)
    ld.add_action(node_velodyne_laserscan)
    ld.add_action(node_ublox_gps)
    ld.add_action(node_rtcm_provider)
    # ld.add_action(node_um7_imu)

    return ld

