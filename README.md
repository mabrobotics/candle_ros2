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

## Example MD service calls - GRIPPER CONTROL

Bring up one or more drives (add → set mode → zero → enable), then open or close the gripper.
`init_devices` applies the same mode to all listed IDs. Gripper open/close use the impedance gains and positions defined in `md_node.hpp` (`IMP_KP`, `IMP_KD`, `IMP_MAX_OUTPUT`, `OPEN_POS`, `CLOSED_POS`).

```bash
# Bring up device 343 in impedance mode
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [343], mode: 'IMPEDANCE'}"

# Close / open gripper
ros2 service call /md/close_gripper candle_ros2/srv/Generic "{device_ids: [343]}"
ros2 service call /md/open_gripper candle_ros2/srv/Generic "{device_ids: [343]}"

# Optional: set impedance gains explicitly (overwritten again by open/close)
ros2 topic pub /md/impedance_command candle_ros2/msg/ImpedanceCmd \
  "{device_ids: [343], kp: [5.0], kd: [0.05], max_output: [3.5]}" --once
```

Individual steps are also available as `/md/add_mds`, `/md/set_mode`, `/md/zero`, and `/md/enable`.

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)
