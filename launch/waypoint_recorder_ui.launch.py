from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='waypoint_recorder',
            executable='waypoint_recorder_ui',
            name='waypoint_recorder_ui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # GUI 앱이므로 DISPLAY 환경 변수가 필요합니다.
            # 원격 SSH 환경이라면: export DISPLAY=:0 후 실행
        )
    ])
