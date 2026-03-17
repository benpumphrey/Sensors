from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = get_package_share_directory('ttt_calibration')
    
    return LaunchDescription([
        # --- DISABLED AUTO-CALIBRATION ---
        # Node(
        #     package='ttt_calibration',
        #     executable='table_calibrator_node',
        #     name='table_calibrator',
        #     output='screen'
        # ),

        # --- RESTORED MANUAL CALIBRATION ---
        # This publishes your hardcoded coordinates from stereo_extrinsic.yaml
        Node(
            package='ttt_calibration',
            executable='tf_broadcaster_node',
            name='tf_broadcaster',
            parameters=[
                os.path.join(config_dir, 'config', 'stereo_extrinsic.yaml')
            ],
            output='screen'
        )
    ])
