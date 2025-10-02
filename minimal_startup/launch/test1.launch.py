import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # Get MAVROS XML launch file (even if it lacks .xml extension)
    mavros_dir = get_package_share_directory('mavros')
    mavros_launch = os.path.join(mavros_dir, 'launch', 'node.launch')
    
    return LaunchDescription([
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node',
            namespace='astro1',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                        os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
            ],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='astro1',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'joy_config.yaml')],
            output='screen'
        ),
        Node(
            package='px4_teleop',
            executable='px4_teleop_node',
            name='px4_teleop_node',
            namespace='astro1',
            parameters=[
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'teleop_config.yaml'),
                os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'sim_obstacles.yaml')
            ],
            output='screen'
        )
    ])
