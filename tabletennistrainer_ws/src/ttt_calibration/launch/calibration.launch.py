from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Auto-Calibrator Node (Finds table based on ArUco markers)
        Node(
            package='ttt_calibration',
            executable='table_calibrator_node',
            name='table_calibrator',
            output='screen'
        )
    ])
