from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='candle_ros2',
            executable='CandleTeleopNode',
            name='candle_keyboard_teleop',
            output='screen',
        )
    ])

