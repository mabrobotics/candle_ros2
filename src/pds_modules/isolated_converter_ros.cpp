#include "candle_ros2/pds_modules/isolated_converter_ros.hpp"

bool IsolatedConverterRos::setup(std::shared_ptr<rclcpp::Node> node,
                                 mab::Pds&                     pds,
                                 mab::socketIndex_E            socket,
                                 const int                     pdsId,
                                 rclcpp::QoS&                  qos,
                                 const std::string&            nodePrefix,
                                 const int                     timerMs)
{
    m_parentNode = node;

    m_isolatedConverter = pds.attachIsolatedConverter(socket);
    if (m_isolatedConverter == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::IsolatedConverterData>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        qos);

    srvEnable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/enable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(
            &IsolatedConverterRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/disable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(
            &IsolatedConverterRos::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

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

void IsolatedConverterRos::cbEnable(
    const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
    std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_isolatedConverter->enable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}
void IsolatedConverterRos::cbDisable(
    const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
    std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_isolatedConverter->disable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}