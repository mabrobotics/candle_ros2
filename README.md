# CANdle ROS2

This repository provides ROS2 interfaces for controlling **MD electric drive controllers** and **PDS power distribution systems** using the [CANdle-SDK](https://github.com/mabrobotics/CANdle-SDK).
It exposes both systems as ROS2 nodes with services and topics for operational control and telemetry.

This package acts as a **runtime control interface**.
For configuration, please use:

➡️ [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle-SDK/CANdleTool.html)

## Features

### MD Node
- Control MD drive controllers
- Publish joint state data
- Accept position, velocity, motion, and impedance commands
- Provide enable/disable/zero/mode setup services

### PDS Node
- Manage PDS devices and their modules
- Monitor modules such as the Control Board, Isolated Converter, Brake Resistor, and Power Stage

## Installation

Go to your ROS2 workspace and clone the repository:

```bash
git clone git@github.com:mabrobotics/candle_ros2.git src/candle_ros2
```

Initialize submodules:

```bash
git -C src/candle_ros2/ submodule update --init --recursive
```

Build:

```bash
colcon build
```

Source the environment:

```bash
source install/setup.bash
```

## Running

### MD Node
```bash
ros2 launch candle_ros2 md_node_launch.py
```

### PDS Node
```bash
ros2 launch candle_ros2 pds_node_launch.py
```

### Both Nodes
```bash
ros2 launch candle_ros2 both_launch.py
```

### Launch arguments

- `bus` — desired communication bus with CANdle device, possible values: `USB` and `SPI` (default: `USB`).
- `data_rate` — data rate of CAN network, possible values: `1M`, `2M`, `5M` and `8M` (default: `1M`).
- `default_qos` — ROS message quality of service for node's publishers, possible values: `Reliable` and `BestEffort` (default: `Reliable`).

Example launch command with custom arguments:
```bash
ros2 launch candle_ros2 md_node_launch.py bus:=SPI data_rate:=5M
```

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)
