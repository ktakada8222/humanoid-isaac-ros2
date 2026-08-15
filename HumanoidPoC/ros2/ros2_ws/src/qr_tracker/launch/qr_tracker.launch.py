from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=["", "/ros2_ws/install/qr_tracker/share/qr_tracker/params/qr_tracker.yaml"],
        description="Path to YAML file with QrTracker parameters",
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package="qr_tracker",
            executable="qr_tracker_node",
            name="qr_tracker_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
