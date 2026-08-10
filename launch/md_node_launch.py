# md_node_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bus_arg = DeclareLaunchArgument(
        "bus",
        default_value="USB",
        description="Bus type: USB or SPI",
    )

    usb_serial_arg = DeclareLaunchArgument(
        "usb_serial",
        default_value="",
        description="Complete USB serial of the CANdle adapter",
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

    gripper_args = [
        DeclareLaunchArgument("joint_name_prefix", default_value="md_"),
        DeclareLaunchArgument("gripper_open_position_rad", default_value="0.0"),
        DeclareLaunchArgument("gripper_closed_position_rad", default_value="0.83"),
        DeclareLaunchArgument("gripper_impedance_kp", default_value="5.0"),
        DeclareLaunchArgument("gripper_impedance_kd", default_value="0.05"),
        DeclareLaunchArgument("gripper_velocity_limit_rad_s", default_value="3.5"),
        DeclareLaunchArgument("gripper_torque_limit_nm", default_value="3.5"),
        DeclareLaunchArgument("init_devices_zero", default_value="false"),
    ]

    bus = LaunchConfiguration("bus")
    usb_serial = LaunchConfiguration("usb_serial")
    data_rate = LaunchConfiguration("data_rate")
    default_qos = LaunchConfiguration("default_qos")

    return LaunchDescription(
        [
            bus_arg,
            usb_serial_arg,
            data_rate_arg,
            default_qos_arg,
            *gripper_args,
            Node(
                package="candle_ros2",
                executable="candle_container",
                output="screen",
                parameters=[
                    {
                        "launch_md_node": True,
                        "launch_pds_node": False,
                        "bus": bus,
                        "usb_serial": ParameterValue(usb_serial, value_type=str),
                        "data_rate": data_rate,
                        "default_qos": default_qos,
                        "joint_name_prefix": LaunchConfiguration("joint_name_prefix"),
                        "gripper_open_position_rad": ParameterValue(
                            LaunchConfiguration("gripper_open_position_rad"), value_type=float
                        ),
                        "gripper_closed_position_rad": ParameterValue(
                            LaunchConfiguration("gripper_closed_position_rad"), value_type=float
                        ),
                        "gripper_impedance_kp": ParameterValue(
                            LaunchConfiguration("gripper_impedance_kp"), value_type=float
                        ),
                        "gripper_impedance_kd": ParameterValue(
                            LaunchConfiguration("gripper_impedance_kd"), value_type=float
                        ),
                        "gripper_velocity_limit_rad_s": ParameterValue(
                            LaunchConfiguration("gripper_velocity_limit_rad_s"), value_type=float
                        ),
                        "gripper_torque_limit_nm": ParameterValue(
                            LaunchConfiguration("gripper_torque_limit_nm"), value_type=float
                        ),
                        "init_devices_zero": ParameterValue(
                            LaunchConfiguration("init_devices_zero"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
