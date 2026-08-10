#pragma once
#include <string>

struct candleParams_S
{
    std::string bus;
    std::string usb_serial;
    std::string data_rate;
    std::string default_qos;
    std::string joint_name_prefix;
    double      gripper_open_position_rad;
    double      gripper_closed_position_rad;
    double      gripper_impedance_kp;
    double      gripper_impedance_kd;
    double      gripper_velocity_limit_rad_s;
    double      gripper_torque_limit_nm;
    bool        init_devices_zero;
};
