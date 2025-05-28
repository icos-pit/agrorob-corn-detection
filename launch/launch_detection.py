from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('corn_yolo_ros_interface'),
        'config',
        'config.yaml'
    )
    ld = LaunchDescription([
        Node(
            package='corn_yolo_ros_interface',
            executable='detection.py',
            name='detector',
            output='screen',
            parameters=[config]
        )
    ])
    return ld

