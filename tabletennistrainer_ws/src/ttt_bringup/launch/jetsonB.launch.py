from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Right Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_right',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'right',
                'width': 640,
                'height': 400,
                'fps': 240,
                'exposure': 6000,
                'analogue_gain': 8000,
            }],
            output='screen'
        ),

        # Right Vision
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_right',
            parameters=[{
                'camera_id': 'right',
                'min_brightness': 10,
                'min_radius': 5,
                'max_radius': 50,
                'show_window': False,
            }],
            output='screen'
        ),
    ])
