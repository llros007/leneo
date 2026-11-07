from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('leneo_description'))
    home = json.loads((share / 'config/home_positions.json').read_text())
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share / 'urdf/leneo_ros.urdf').read_text()}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'zeros.' + name: value for name, value in home.items()}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/display.rviz')]),
    ])
