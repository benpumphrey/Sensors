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
                'width': 640,
                'height': 400,
                'fps': 240,
                'show_window': False  # Enable window
            }],
            output='screen'
        ),
        # Left Vision Tracker
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='vision_left',
            parameters=[{
                'camera_id': 'left',
                'debug_mode': True,
                'blur_size': 5,
                'min_brightness': 30,
                'min_radius': 3
            }],
            output='screen'
        )
    ]) 
