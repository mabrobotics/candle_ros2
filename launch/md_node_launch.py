# md_node_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bus_arg = DeclareLaunchArgument(
        "bus",
        default_value="USB",
        description="Bus type: USB or SPI",
    )

    data_rate_arg = DeclareLaunchArgument(
        "data_rate",
        default_value="1M",
        description="Data rate: 1M, 2M, 5M or 8M",
    )

    bus = LaunchConfiguration("bus")
    data_rate = LaunchConfiguration("data_rate")

    return LaunchDescription(
        [
            bus_arg,
            data_rate_arg,
            Node(
                package="candle_ros2",
                executable="candle_container",
                output="screen",
                parameters=[
                    {
                        "launch_md_node": True,
                        "launch_pds_node": False,
                        "data_rate": data_rate,
                        "bus": bus,
                    }
                ],
            ),
        ]
    )
