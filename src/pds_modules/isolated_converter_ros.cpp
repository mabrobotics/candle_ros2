#include "candle_ros2/pds_modules/isolated_converter_ros.hpp"

bool IsolatedConverterRos::setup(rclcpp::Node*      node,
                                 mab::Pds&          pds,
                                 mab::socketIndex_E socket,
                                 const int          pdsId,
                                 rclcpp::QoS&       qos,
                                 const int          timerMs)
{
    m_parentNode = node;

    m_isolatedConverter = pds.attachIsolatedConverter(socket);
    if (m_isolatedConverter == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::IsolatedConverterData>(
        topicName(pdsId, MODULE_NAME, socket), qos);

    createEnableServices(m_isolatedConverter.get(), pdsId, MODULE_NAME, socket);

    tmrPub = m_parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                             std::bind(&IsolatedConverterRos::publishStatus, this));

    return true;
}

void IsolatedConverterRos::publishStatus()
{
    auto msg = candle_ros2::msg::IsolatedConverterData();

    msg.header.stamp = m_parentNode->get_clock()->now();

    m_isolatedConverter->getEnabled(msg.enabled);
    m_isolatedConverter->getOutputVoltage(msg.output_voltage);
    m_isolatedConverter->getLoadCurrent(msg.load_current);
    /* Power and energy reads are not implemented yet */
    // m_isolatedConverter->getPower(msg.power);
    // m_isolatedConverter->getEnergy(msg.energy);
    m_isolatedConverter->getOcdLevel(msg.ocd_level);
    m_isolatedConverter->getOcdDelay(msg.ocd_delay);
    m_isolatedConverter->getTemperature(msg.temperature);
    m_isolatedConverter->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}
