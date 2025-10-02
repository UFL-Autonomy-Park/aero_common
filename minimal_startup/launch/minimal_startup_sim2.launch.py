import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # Get MAVROS XML launch file (even if it lacks .xml extension)
    mavros_dir = get_package_share_directory('mavros')
    mavros_launch = os.path.join(mavros_dir, 'launch', 'node.launch')
    
    # Define the correct path to QGroundControl
    qgc_path = '/home/autonomypark/QGroundControl.AppImage'
    
    return LaunchDescription([
        
        # Original nodes from minimal_startup.launch.py
        Node(
            package='autonomy_park_viz',
            executable='autonomy_park_viz_node',
            name='autonomy_park_viz_node',
            namespace='autonomy_park_viz',
            parameters=[os.path.join(get_package_share_directory('autonomy_park_viz'), 'param', 'park_geometry.yaml')],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('autonomy_park_viz'), 'rviz', 'autonomy_park.rviz')]
        ),
        
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node',
            namespace='astro_sim_2',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                        os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
            ],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='astro_sim_2',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'joy_config.yaml')],
            output='screen'
        )
    ])