# Candle ROS2 software



## MD ROS2 Node

This node handles the communication between MAB's MD drive controllers in ROS2 environment. The node was designed to act as 
operational endpoint - to control the drives and get information from them, thus it is not capable of configuring the drives, 
for this use [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/software_package/CANdleSDK/CANdleTool.html).

### Available services and topics

The node normally communicates via services for setup, and via topics for regular data transfers.
Services are: 
- /md/add_mds
- /md/disable
- /md/enable
- /md/set_mode
- /md/zero

Topics subscribed by the node are:
- /md/motion_command
- /md/position_command
- /md/velocity_command
- /md/impedance_command

Topic published by the node is:
- /md/joint_states

## PDS ROS2 Node


## Quick startup guide

Please find a detailed startup guide in the [MD80 x CANdle manual](https://www.mabrobotics.pl/servos)