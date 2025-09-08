// candle_factory.hpp
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "mab/Candle.hpp"
#include "candle_ros2/utils/candle_params.hpp"

inline candleParams_S readParams(const rclcpp::Node::SharedPtr& node)
{
    node->declare_parameter<std::string>("data_rate", "1M");
    node->declare_parameter<std::string>("bus", "USB");
    node->declare_parameter<int>("sample_rate", 1000);
    node->declare_parameter<double>("timeout_sec", 0.1);

    candleParams_S params;
    params.data_rate   = node->get_parameter("data_rate").as_string();
    params.bus         = node->get_parameter("bus").as_string();
    params.sample_rate = node->get_parameter("sample_rate").as_int();
    params.timeout_sec = node->get_parameter("timeout_sec").as_double();
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
