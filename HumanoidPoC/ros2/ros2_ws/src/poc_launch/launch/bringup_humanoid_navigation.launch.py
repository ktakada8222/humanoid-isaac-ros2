from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- pseudo_odometry_node（最初に起動） ---
    pseudo_odom = Node(
        package='poc_utils',
        executable='pseudo_odometry_node',
        name='pseudo_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # hdl_localization
    hdl_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('hdl_localization'), 'launch', 'hdl_localization.launch.py')
        ]),
        launch_arguments={
            'globalmap_pcd': '/volume/slam/glim/maps/lightwheel_map_with_intensity.pcd',
            'use_pcd_map': 'true',
            'use_imu': 'true',
            'points_topic': '/lidar/points',
            'imu_topic': '/imu/data',
            'use_sim_time': 'true'
        }.items()
    )

    # pointcloud_to_laserscan
    pc2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=['/volume/pc2_to_scan_cfg/pc2_to_scan.yaml'],
        remappings=[
            ('cloud_in', '/lidar/points'),
            ('scan', '/scan')
        ]
    )

    # poc_launch
    poc_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('poc_launch'), 'launch', 'bringup_no_amcl.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/volume/maps/lightwheel_map/lightwheel_map_occgrid.yaml',
            'params_file': '/volume/nav2_cfg/nav2_params_maxV1.0_humble.yaml'
        }.items()
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/volume/rviz2_cfg/humanoid_navigation_by_uzawa.rviz']
    )

    return LaunchDescription([
        # pseudo_odom,
        hdl_localization,
        pc2scan,
        poc_bringup,
        rviz
    ])

