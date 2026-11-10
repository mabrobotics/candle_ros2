#include "candle_ros2/pds_modules/brake_resistor_ros.hpp"

bool BrakeResistorRos::setup(rclcpp::Node*      node,
                             mab::Pds&          pds,
                             mab::socketIndex_E socket,
                             const int          pdsId,
                             rclcpp::QoS&       qos,
                             const int          timerMs)
{
    m_parentNode = node;

    m_brakeResistor = pds.attachBrakeResistor(socket);
    if (m_brakeResistor == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::BrakeResistorData>(
        topicName(pdsId, MODULE_NAME, socket), qos);

    createEnableServices(m_brakeResistor.get(), pdsId, MODULE_NAME, socket);

    tmrPub = m_parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                             std::bind(&BrakeResistorRos::publishStatus, this));

    return true;
}

void BrakeResistorRos::publishStatus()
{
    auto msg = candle_ros2::msg::BrakeResistorData();

    msg.header.stamp = m_parentNode->get_clock()->now();

    m_brakeResistor->getEnabled(msg.enabled);
    m_brakeResistor->getTemperature(msg.temperature);
    m_brakeResistor->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}
