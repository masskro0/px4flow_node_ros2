import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "mavlink_serial_client"
    exe = "mavlink_serial_client"

    config = os.path.join(
        get_package_share_directory(pkg),
        'default.yaml'
    )
    return LaunchDescription([
        Node(
            package=pkg,
            namespace=pkg,
            executable=exe,
            name=exe,
            parameters=[config]
        )
    ])
