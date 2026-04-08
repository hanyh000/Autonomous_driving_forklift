from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fp_pkg',
            executable='db_astar',
            output='screen',
            prefix='xterm -e',          # 별도 터미널 창에서 실행 → 키보드 입력 가능
            emulate_tty=True,
        ),
        Node(
            package='fp_pkg',
            executable='unloading_node',
            output='screen',
        ),
        Node(
            package='fp_pkg',
            executable='yolo_slow_node',
            output='screen',
        ),
    ])