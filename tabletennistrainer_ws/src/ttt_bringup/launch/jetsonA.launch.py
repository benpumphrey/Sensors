from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Left Camera - 120 FPS with window
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'width': 1280,
                'height': 720,
                'fps': 120,
                'show_window': True  # Enable window
            }],
            output='screen'
        )
    ]) 
