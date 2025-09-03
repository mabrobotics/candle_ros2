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
        nodePrefix + std::to_string(pdsId) + "/" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        10);

    srvEnable = parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + std::to_string(pdsId) + "/enable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(&BrakeResistorRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = parentNode->create_service<candle_ros2::srv::GenericPds>(
        nodePrefix + std::to_string(pdsId) + "/disable_" + std::string(MODULE_NAME) + "_" +
            std::to_string(static_cast<int>(socket)),
        std::bind(&BrakeResistorRos::cbEnable, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                           std::bind(&BrakeResistorRos::publishStatus, this));

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

void BrakeResistorRos::cbEnable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                                std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    if (brakeResistor->enable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}
void BrakeResistorRos::cbDisable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                                 std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp)
{
    if (brakeResistor->disable() != mab::PdsModule::error_E::OK)
        rsp->success.push_back(false);
    else
        rsp->success.push_back(true);
    return;
}