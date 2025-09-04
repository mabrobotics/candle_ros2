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
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        10);

    srvEnable = parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/enable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(
            &IsolatedConverterRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/disable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(
            &IsolatedConverterRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                           std::bind(&IsolatedConverterRos::publishStatus, this));

    return true;
}

void IsolatedConverterRos::publishStatus()
{
    auto msg = candle_ros2::msg::IsolatedConverterData();

    msg.header.stamp = parentNode->get_clock()->now();

    isolatedConverter->getEnabled(msg.enabled);
    isolatedConverter->getOutputVoltage(msg.output_voltage);
    isolatedConverter->getLoadCurrent(msg.load_current);
    /* Power and energy reads are not implemented yet */
    // isolatedConverter->getPower(msg.power);
    // isolatedConverter->getEnergy(msg.energy);
    isolatedConverter->getOcdLevel(msg.ocd_level);
    isolatedConverter->getOcdDelay(msg.ocd_delay);
    isolatedConverter->getTemperature(msg.temperature);
    isolatedConverter->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}

void IsolatedConverterRos::cbEnable(
    const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
    std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    if (isolatedConverter->enable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}
void IsolatedConverterRos::cbDisable(
    const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
    std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    if (isolatedConverter->disable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}