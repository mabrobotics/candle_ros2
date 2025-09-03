#include "candle_ros2/pds_modules/brake_resistor_ros.hpp"

bool BrakeResistorRos::setup(std::shared_ptr<rclcpp::Node> node,
                             mab::Pds&                     pds,
                             mab::socketIndex_E            socket,
                             const int                     pdsId,
                             const std::string&            nodePrefix,
                             const int                     timerMs)
{
    parentNode = node;

    brakeResistor = pds.attachBrakeResistor(socket);
    if (brakeResistor == nullptr)
        return false;

    pubData = parentNode->create_publisher<candle_ros2::msg::BrakeResistorData>(
        nodePrefix + std::to_string(pdsId) + "/brake_resistor_" +
            std::to_string(static_cast<int>(socket)),
        10);

    tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                           [this]() { this->publishStatus(); });

    return true;
}

void BrakeResistorRos::publishStatus()
{
    auto msg = candle_ros2::msg::BrakeResistorData();

    msg.header.stamp = parentNode->get_clock()->now();

    brakeResistor->getEnabled(msg.enabled);
    brakeResistor->getTemperature(msg.temperature);
    brakeResistor->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}