# CANdle-SDK ROS2 Software

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

This node is also designed as an operational endpoint to control PDS modules and get data about all modules states. It **does not configure the PDS device**.

### Services and Topics

Available services are:

- `/pds/add_pds`
- `/pds/reboot_pds`
- `/pds/shutdown_pds`

After adding a PDS with a specified ID, this node automatically creates topics and services for all its modules. Topic naming convention:

- /pds/id_**\<id\>**/**\<module_name\>**_**\<socket_number\>**

**Example:** For PDS ID 100 with an *Isolated Converter* on socket 1:

- `/pds/id_100/ctrl`
- `/pds/id_100/ic_1`

There always will be `ctrl` module beacause it's the PDS control board. For more information about every module topic read this page: [PDS Modules Topics](docs/PdsModule.md)  

Every module has **enable** and **disable** services, which follow a similar naming convention:

- `/pds/id_100/disable_ic_1`
- `/pds/id_100/enable_ic_1`

## Build

Go to your ROS2 workspace directory. Clone the [repository](https://github.com/mabrobotics/candle_ros2) into the `src/` directory of the workspace:

```bash
git clone git@github.com:mabrobotics/candle_ros2.git src/candle_ros2
```
Initialize submodules:

```bash
git -C src/candle_ros2/ submodule update --init --recursive
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