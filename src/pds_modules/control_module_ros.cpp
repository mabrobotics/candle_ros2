#include "candle_ros2/pds_modules/control_module_ros.hpp"

bool ControlModuleRos::setup(std::shared_ptr<rclcpp::Node> node,
                             mab::Pds&                     pds,
                             mab::socketIndex_E            socket,
                             const int                     pdsId,
                             const std::string&            nodePrefix,
                             const int                     timerMs)
{
    parentNode    = node;
    controlModule = &pds;
    (void)socket;

    if (controlModule == nullptr)
        return false;

    pubData = parentNode->create_publisher<candle_ros2::msg::ControlModuleData>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME), 10);

    tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                           std::bind(&ControlModuleRos::publishStatus, this));

    return true;
}

void ControlModuleRos::publishStatus()
{
    auto msg = candle_ros2::msg::ControlModuleData();

    msg.header.stamp = parentNode->get_clock()->now();

    controlModule->getBusVoltage(msg.bus_voltage);
    controlModule->getBatteryVoltageLevels(msg.battery_voltage_level_1,
                                           msg.battery_voltage_level_2);
    controlModule->getBrakeResistorTriggerVoltage(msg.brake_trigger_voltage);
    controlModule->getTemperature(msg.temperature);
    controlModule->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}
