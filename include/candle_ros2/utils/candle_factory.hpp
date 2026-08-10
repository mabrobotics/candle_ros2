#pragma once
#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"

/* CANdle-SDK */
#include "candle.hpp"

inline candleParams_S readParams(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<std::string>("data_rate", "1M");
    node->declare_parameter<std::string>("bus", "USB");
    node->declare_parameter<std::string>("usb_serial", "");
    node->declare_parameter<std::string>("default_qos", "Reliable");
    node->declare_parameter<std::string>("joint_name_prefix", "md_");
    node->declare_parameter<double>("gripper_open_position_rad", 0.0);
    node->declare_parameter<double>("gripper_closed_position_rad", 0.83);
    node->declare_parameter<double>("gripper_impedance_kp", 5.0);
    node->declare_parameter<double>("gripper_impedance_kd", 0.05);
    node->declare_parameter<double>("gripper_velocity_limit_rad_s", 3.5);
    node->declare_parameter<double>("gripper_torque_limit_nm", 3.5);
    node->declare_parameter<bool>("init_devices_zero", false);

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.usb_serial  = node->get_parameter("usb_serial").as_string();
    params.default_qos = node->get_parameter("default_qos").as_string();
    params.joint_name_prefix = node->get_parameter("joint_name_prefix").as_string();
    params.gripper_open_position_rad =
        node->get_parameter("gripper_open_position_rad").as_double();
    params.gripper_closed_position_rad =
        node->get_parameter("gripper_closed_position_rad").as_double();
    params.gripper_impedance_kp = node->get_parameter("gripper_impedance_kp").as_double();
    params.gripper_impedance_kd = node->get_parameter("gripper_impedance_kd").as_double();
    params.gripper_velocity_limit_rad_s =
        node->get_parameter("gripper_velocity_limit_rad_s").as_double();
    params.gripper_torque_limit_nm =
        node->get_parameter("gripper_torque_limit_nm").as_double();
    params.init_devices_zero = node->get_parameter("init_devices_zero").as_bool();
    return params;
}

inline std::shared_ptr<mab::Candle> createCandle(const candleParams_S& params)
{
    auto dataRate = mab::CANdleDatarate_E::CAN_DATARATE_1M;
    auto bus      = mab::candleTypes::busTypes_t::USB;

    if (params.data_rate == "2M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_2M;
    else if (params.data_rate == "5M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_5M;
    else if (params.data_rate == "8M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_8M;

    if (params.bus == "SPI")
        bus = mab::candleTypes::busTypes_t::SPI;

    if (bus == mab::candleTypes::busTypes_t::USB)
    {
        std::unique_ptr<mab::I_CommunicationInterface> usb =
            std::make_unique<mab::USB>(mab::Candle::CANDLE_VID,
                                       mab::Candle::CANDLE_PID,
                                       params.usb_serial);
        if (usb->connect() != mab::I_CommunicationInterface::Error_t::OK)
            throw std::runtime_error("Could not connect selected USB device!");
        return std::shared_ptr<mab::Candle>(
            mab::attachCandle(dataRate, std::move(usb)));
    }

    return std::shared_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));
}
