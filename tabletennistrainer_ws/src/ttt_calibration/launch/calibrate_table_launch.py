from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=""),
        LogInfo(msg="========================================"),
        LogInfo(msg="  TABLE CORNER CALIBRATION"),
        LogInfo(msg="========================================"),
        LogInfo(msg=""),
        LogInfo(msg="Place ArUco markers at table corners:"),
        LogInfo(msg="  ID 0: Front-left"),  # FIXED: Added closing parenthesis
        LogInfo(msg="  ID 1: Front-right"),
        LogInfo(msg="  ID 2: Back-right"),
        LogInfo(msg="  ID 3: Back-left"),
        LogInfo(msg=""),
        LogInfo(msg="Press 'c' to calibrate, 's' to save"),
        LogInfo(msg="========================================"),
        
        # Camera
        Node(
            package='ttt_camera',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'device': '/dev/video0',
                'camera_id': 'left',
                'width': 640,
                'height': 400,
                'fps': 60,
            }],
            output='screen'
        ),
        
        # ArUco Calibration
        Node(
            package='ttt_calibration',
            executable='aruco_calibrate_camera',
            name='aruco_calibrate_left',
            parameters=[{
                'camera_id': 'left',
                'marker_size': 0.10,  # 10cm markers
            }],
            output='screen'
        )
    ])