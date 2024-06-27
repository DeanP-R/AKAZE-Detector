import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory('akaze_detector')
    rqt_config_file = os.path.join(
        package_share_directory,
        'config',
        'akaze_view.perspective'
    )
    
    return LaunchDescription([
        Node(
            package='akaze_detector',
            executable='akaze_detector_node',
            name='akaze',
            output='screen'
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            arguments=['--perspective-file', rqt_config_file]
        )
    ])