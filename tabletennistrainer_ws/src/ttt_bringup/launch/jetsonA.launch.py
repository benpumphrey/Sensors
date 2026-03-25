from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    calibration_dir = get_package_share_directory('ttt_calibration')
    moveit_config_dir = get_package_share_directory('ttt_control')

    return LaunchDescription([
        # Launch calibration first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_dir, 'launch', 'calibration.launch.py')
            )
        ),

        # Robot State Publisher (publishes URDF + joint TF transforms)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py')
            )
        ),

        # MoveIt move_group (IK solver + motion planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
            )
        ),

        # Left Camera
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
                'exposure': 3000,
                'analogue_gain': 8000,
            }],
            output='screen'
        ),

        # Left Vision
        Node(
            package='ttt_vision',
            executable='vision_node',
            name='ball_detector_left',
            parameters=[{
                'camera_id': 'left',
                'min_brightness': 5,
                'min_radius': 5,
                'max_radius': 50,
                'show_window': False,
            }],
            output='screen'
        ),

        # Stereo Vision (receives right detection from Jetson B via DDS)
        Node(
            package='ttt_stereo',
            executable='stereo_node',
            parameters=[{
                'baseline_m': 1.5,
                'fx': 224.1,
                'fy': 200.0,
                'cx': 320.0,
                'cy': 200.0,
                'max_sync_age_ms': 100,
            }],
        ),

        # Trajectory Prediction
        Node(
            package='ttt_trajectory',
            executable='trajectory_node',
            name='trajectory_node',
            parameters=[{
                'lookahead_ms': 250,
                'min_samples': 5,
                'max_samples': 20,
                'gravity': 9.81,
                'table_y': -0.5,
                'restitution': 0.85,
            }],
            output='screen'
        ),

        # Hardware Bridge (UDP to STM32 at 192.168.1.100)
        Node(
            package='ttt_hardware',
            executable='hardware_node',
            parameters=[{
                'stm_ip': '192.168.1.100',
                'stm_port': 5000,
                'joint_topic': '/joint_states',
            }],
            output='screen'
        ),
    ])
