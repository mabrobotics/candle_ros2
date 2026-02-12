#include "candle_ros2/pds_modules/brake_resistor_ros.hpp"

bool BrakeResistorRos::setup(std::shared_ptr<rclcpp::Node> node,
                             mab::Pds&                     pds,
                             mab::socketIndex_E            socket,
                             const int                     pdsId,
                             rclcpp::QoS&                  qos,
                             const std::string&            nodePrefix,
                             const int                     timerMs)
{
    m_parentNode = node;

    m_brakeResistor = pds.attachBrakeResistor(socket);
    if (m_brakeResistor == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::BrakeResistorData>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        qos);

    srvEnable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/enable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(&BrakeResistorRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/disable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(
            &BrakeResistorRos::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

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

void BrakeResistorRos::cbEnable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                                std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_brakeResistor->enable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}
void BrakeResistorRos::cbDisable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                                 std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_brakeResistor->disable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}