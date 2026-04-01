from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fp_pkg',
            executable='db_astar',
            output='screen'
        ),
        Node(
            package='fp_pkg',
            executable='unloading_node',
            output='screen'
        ),
    ])