#include "candle_ros2/pds_modules/power_stage_ros.hpp"

bool PowerStageRos::setup(std::shared_ptr<rclcpp::Node> node,
                          mab::Pds&                     pds,
                          mab::socketIndex_E            socket,
                          const int                     pdsId,
                          rclcpp::QoS&                  qos,
                          const std::string&            nodePrefix,
                          const int                     timerMs)
{
    m_parentNode = node;

    m_powerStage = pds.attachPowerStage(socket);
    if (m_powerStage == nullptr)
        return false;

    pubData = m_parentNode->create_publisher<candle_ros2::msg::PowerStageData>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        qos);

    srvEnable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/enable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(&PowerStageRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));

    srvDisable = m_parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + "id_" + std::to_string(pdsId) + "/disable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(&PowerStageRos::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = m_parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                             std::bind(&PowerStageRos::publishStatus, this));

    return true;
}

void PowerStageRos::publishStatus()
{
    auto msg = candle_ros2::msg::PowerStageData();

    msg.header.stamp = m_parentNode->get_clock()->now();

    m_powerStage->getEnabled(msg.enabled);

    mab::socketIndex_E sck;
    m_powerStage->getBindBrakeResistor(sck);
    msg.brake_resistor_socket = static_cast<uint8_t>(sck);

    m_powerStage->getBrakeResistorTriggerVoltage(msg.trigger_voltage);
    m_powerStage->getOutputVoltage(msg.output_voltage);
    m_powerStage->getAutostart(msg.autostart);
    m_powerStage->getLoadCurrent(msg.load_current);
    m_powerStage->getPower(msg.power);
    m_powerStage->getTotalDeliveredEnergy(msg.energy);
    m_powerStage->getOcdLevel(msg.ocd_level);
    m_powerStage->getOcdDelay(msg.ocd_delay);
    m_powerStage->getTemperature(msg.temperature);
    m_powerStage->getTemperatureLimit(msg.temperature_limit);

    pubData->publish(msg);
}

void PowerStageRos::cbEnable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                             std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_powerStage->enable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}
void PowerStageRos::cbDisable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                              std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    (void)req;
    if (m_powerStage->disable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}