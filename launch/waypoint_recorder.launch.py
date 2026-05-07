from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'save_path',
            default_value=os.path.expanduser('~/waypoints'),
            description='Directory to save waypoint YAML files'
        ),
        DeclareLaunchArgument(
            'filename',
            default_value='',
            description='Filename for saved waypoints (empty = auto timestamp)'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame ID'
        ),
        DeclareLaunchArgument(
            'auto_save',
            default_value='true',
            description='Auto-save on every new waypoint'
        ),

        Node(
            package='waypoint_recorder',
            executable='waypoint_recorder',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'save_path': LaunchConfiguration('save_path'),
                'filename': LaunchConfiguration('filename'),
                'map_frame': LaunchConfiguration('map_frame'),
                'auto_save': LaunchConfiguration('auto_save'),
            }]
        )
    ])