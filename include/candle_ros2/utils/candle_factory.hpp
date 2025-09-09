#pragma once
#include "rclcpp/rclcpp.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"

/* CANdle-SDK */
#include "candle.hpp"

inline candleParams_S readParams(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<std::string>("data_rate", "1M");
    node->declare_parameter<std::string>("bus", "USB");
    node->declare_parameter<std::string>("default_qos", "Reliable");

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.default_qos = node->get_parameter("default_qos").as_string();
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

    return std::shared_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));
}
