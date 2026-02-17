from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'widith': 1280,
                'height': 720,
                'fps': 120
            }],
            output='screen'
        )
    ])