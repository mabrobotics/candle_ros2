# PDS Node Topics

This page contains basic information about PDS modules topics. For more information check: [PDS documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/PDS/intro.html)

## Control module topic

Topic name: `/pds/id_<id_number>/ctrl`

Published data:

| Name                    |      Type       |
| ----------------------- | :-------------: |
| header                  | std_msgs/Header |
| bus_voltage             |     uint32      |
| battery_voltage_level_1 |     uint32      |
| battery_voltage_level_2 |     uint32      |
| brake_trigger_voltage   |     uint32      |
| temperature             |     float32     |
| temperature_limit       |     float32     |

## Brake Resistior module topic

Topic name: `/pds/id_<id_number>/br_<socket_number>`

Published data:

| Name              |      Type       |
| ----------------- | :-------------: |
| header            | std_msgs/Header |
| enabled           |      bool       |
| temperature       |     float32     |
| temperature_limit |     float32     |

## Isolated Converter module topic

Topic name: `/pds/id_<id_number>/ic_<socket_number>`

Published data:

| Name              |      Type       |
| ----------------- | :-------------: |
| header            | std_msgs/Header |
| enabled           |      bool       |
| output_voltage    |     uint32      |
| load_current      |      int32      |
| ocd_level         |     uint32      |
| ocd_delay         |     uint32      |
| temperature       |     float32     |
| temperature_limit |     float32     |



## Power Stage module topic

Topic name: `/pds/id_<id_number>/ps_<socket_number>`

Published data:

| Name                  |      Type       |
| --------------------- | :-------------: |
| header                | std_msgs/Header |
| enabled               |      bool       |
| brake_resistor_socket |      uint8      |
| trigger_voltage       |     uint32      |
| output_voltage        |     uint32      |
| autostart             |      bool       |
| load_current          |      int32      |
| power                 |      int32      |
| energy                |     uint32      |
| ocd_level             |     uint32      |
| ocd_delay             |     uint32      |
| temperature           |     float32     |
| temperature_limit     |     float32     |