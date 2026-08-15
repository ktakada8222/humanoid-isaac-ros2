from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pc2_to_scan",
            output="screen",
            parameters=[{
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": -0.54,
                "max_height": 0.5,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 10.0,   # LiDARの測距に合わせる
                "use_inf": True
            }],
            remappings=[
                ("cloud_in", "/livox/lidar"),
                ("scan", "/scan")
            ]
        )
    ])
