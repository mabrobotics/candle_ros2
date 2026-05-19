#pragma once
#include "rclcpp/rclcpp.hpp"

/* Services */
#include "candle_ros2/srv/add_devices.hpp"
#include "candle_ros2/srv/generic.hpp"

/* PDS ROS2 modules */
#include "candle_ros2/pds_modules/brake_resistor_ros.hpp"
#include "candle_ros2/pds_modules/control_module_ros.hpp"
#include "candle_ros2/pds_modules/isolated_converter_ros.hpp"
#include "candle_ros2/pds_modules/power_stage_ros.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"
#include "candle_ros2/utils/pds_instance.hpp"

/* CANdle-SDK */
#include "candle.hpp"
#include "pds.hpp"

class PdsNode : public rclcpp::Node
{
  public:
    PdsNode(const std::string&           ns,
            const rclcpp::NodeOptions&   options,
            std::shared_ptr<mab::Candle> candle,
            const candleParams_S&        params);
    ~PdsNode();

  private:
    std::shared_ptr<mab::Candle> m_candle;
    std::vector<pdsInstance_S>   m_pdsList;

    static constexpr int PUB_TIMER_MS = 100;  // 10 Hz

    rclcpp::QoS m_defaultQoS;

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr srvAddPds;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr    srvReboot;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr    srvShutdown;

    rclcpp::TimerBase::SharedPtr tmrPub;

    void cbAddPds(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                  std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp);
    void cbReboot(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                  std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbShutdown(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);

    std::unique_ptr<I_BaseModuleRos> createModule(mab::moduleType_E type);
};
