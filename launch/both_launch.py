# both_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="candle_ros2",
                executable="candle_container",
                name="candle_container",
                output="screen",
                parameters=[
                    {
                        "launch_md_node": True,
                        "launch_pds_node": True,
                        "data_rate": "1M",
                        "bus": "USB",
                    }
                ],
            )
        ]
    )
