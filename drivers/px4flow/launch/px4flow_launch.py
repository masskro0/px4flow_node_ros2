import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = 'px4flow'
    exe = 'px4flow'
    config_file = "default.yaml"

    return LaunchDescription([
        Node(
            package=pkg,
            executable=exe,
            namespace=pkg,
            name=exe,
            parameters=[os.path.join(get_package_share_directory(pkg), config_file)]
        )
    ])
