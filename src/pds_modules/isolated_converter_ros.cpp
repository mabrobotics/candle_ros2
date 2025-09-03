#include "candle_ros2/pds_modules/isolated_converter_ros.hpp"

bool IsolatedConverterRos::setup(std::shared_ptr<rclcpp::Node> node,
                                 mab::Pds&                     pds,
                                 mab::socketIndex_E            socket,
                                 const int                     pdsId,
                                 const std::string&            nodePrefix,
                                 const int                     timerMs)
{
    parentNode = node;

    isolatedConverter = pds.attachIsolatedConverter(socket);
    if (isolatedConverter == nullptr)
        return false;

    pubData = parentNode->create_publisher<candle_ros2::msg::IsolatedConverterData>(
        nodePrefix + std::to_string(pdsId) + "/isolated_converter_" +
            std::to_string(static_cast<int>(socket)),
        10);

    tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                           [this]() { this->publishStatus(); });

    return true;
}

void IsolatedConverterRos::publishStatus()
{
    auto msg = candle_ros2::msg::IsolatedConverterData();

    msg.header.stamp = parentNode->get_clock()->now();

    isolatedConverter->getEnabled(msg.enabled);
    isolatedConverter->getOutputVoltage(msg.output_voltage);
    isolatedConverter->getLoadCurrent(msg.load_current);
    isolatedConverter->getPower(msg.power);
    isolatedConverter->getEnergy(msg.energy);
    isolatedConverter->getOcdLevel(msg.ocd_level);
    isolatedConverter->getOcdDelay(msg.ocd_delay);
    isolatedConverter->getTemperature(msg.temperature);
    isolatedConverter->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}