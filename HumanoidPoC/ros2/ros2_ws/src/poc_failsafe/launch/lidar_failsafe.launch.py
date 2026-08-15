from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="poc_failsafe",
            executable="lidar_failsafe_node",
            name="lidar_failsafe_node",
            output="screen",
            parameters=[{
                "timeout_sec": 5.0  # LIDARが5秒途絶したら停止
            }],
        ),
    ])
