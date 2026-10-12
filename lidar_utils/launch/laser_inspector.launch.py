from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lidar_utils',
            executable='laser_inspector',
            name='laser_inspector_node',
            output='screen'
        )
    ])
