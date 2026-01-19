# both_launch.py
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

    default_qos_arg = DeclareLaunchArgument(
        "default_qos",
        default_value="Reliable",
        description='Quality of Service: "BestEffort" or "Reliable"',
    )

    bus = LaunchConfiguration("bus")
    data_rate = LaunchConfiguration("data_rate")
    default_qos = LaunchConfiguration("default_qos")

    return LaunchDescription(
        [
            bus_arg,
            data_rate_arg,
            default_qos_arg,
            Node(
                package="candle_ros2",
                executable="candle_container",
                output="screen",
                parameters=[
                    {
                        "launch_md_node": True,
                        "launch_pds_node": True,
                        "bus": bus,
                        "data_rate": data_rate,
                        "default_qos": default_qos,
                    }
                ],
            ),
        ]
    )
