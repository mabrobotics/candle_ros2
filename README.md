# Candle ROS2 Software

This repository provides two main nodes: one for controlling MD electric drive controllers and another for communicating with and controlling PDS devices.

## MD ROS2 Node

This node manages communication between MAB's MD drive controllers in a ROS2 environment. It is designed as an **operational endpoint** to control drives and retrieve information. It **does not configure drives**; for configuration, use [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/software_package/CANdleSDK/CANdleTool.html).

### Services and Topics

The node communicates via **services** for setup and **topics** for regular data transfer.  

**Services:**
- `/md/add_mds`
- `/md/disable`
- `/md/enable`
- `/md/set_mode`
- `/md/zero`

**Subscribed topics:**
- `/md/motion_command`
- `/md/position_command`
- `/md/velocity_command`
- `/md/impedance_command`

**Published topic:**
- `/md/joint_states`

## PDS ROS2 Node

After adding a PDS with a specified ID, this node automatically creates topics for all its modules. Topic naming convention:

- /pds/id_**\<id\>**/**\<module_name\>**_**\<socket_number\>**

**Example:** For PDS ID 100 with an *Isolated Converter* on socket 1:

- `/pds/id_100/control`
- `/pds/id_100/isolated_converter_1`

The `control` module publishes data from the PDS control board.  

For other modules, **enable** and **disable** services follow a similar convention:

- `/pds/id_100/disable_isolated_converter_1`
- `/pds/id_100/enable_isolated_converter_1`

Other available services:

- `/pds/reboot_pds`
- `/pds/shutdown_pds`

## Build

Clone the [repository](https://github.com/mabrobotics/candle_ros2) into the `src/` directory of your ROS2 workspace:

```bash
git clone <repo_url> src/candle_ros2
```
Initialize submodules:

```bash
git submodule update --init --recursive
```

Build the workspace:

```bash
colcon build
```

Source the environment:

```bash
source install/setup.bash
```

And you are ready to run the nodes.


## Quick startup guide

For detailed instructions, see the [MD x CANdle manual](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)