from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'system_localization.launch.py')
            )
        ),
        Node(
            package='tag_nav_stub',
            executable='mission_manager_node',
            name='mission_manager',
            parameters=[{'scenario': 'A'}]
        )
    ])
