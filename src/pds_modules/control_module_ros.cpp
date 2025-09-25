#include "candle_ros2/pds_modules/control_module_ros.hpp"

bool ControlModuleRos::setup(std::shared_ptr<rclcpp::Node> node,
                             mab::Pds&                     pds,
                             mab::socketIndex_E            socket,
                             const int                     pdsId,
                             rclcpp::QoS&                  qos,
                             const std::string&            nodePrefix,
                             const int                     timerMs)
{
    m_parentNode    = node;
    m_controlModule = &pds;
    (void)socket;

    if (m_controlModule == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::ControlModuleData>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME), qos);

    tmrPub = m_parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                             std::bind(&ControlModuleRos::publishStatus, this));

    return true;
}

void ControlModuleRos::publishStatus()
{
    auto msg = candle_ros2::msg::ControlModuleData();

    msg.header.stamp = m_parentNode->get_clock()->now();

    m_controlModule->getBusVoltage(msg.bus_voltage);
    m_controlModule->getBatteryVoltageLevels(msg.battery_voltage_level_1,
                                             msg.battery_voltage_level_2);
    m_controlModule->getBrakeResistorTriggerVoltage(msg.brake_trigger_voltage);
    m_controlModule->getTemperature(msg.temperature);
    m_controlModule->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}
