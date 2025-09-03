#pragma once
#include "rclcpp/rclcpp.hpp"

/* Services */
#include "candle_ros2/srv/add_devices.hpp"
#include "candle_ros2/srv/generic.hpp"

/* PDS with modules container */
#include "candle_ros2/pds/pds_instance.hpp"

/* PDS ROS2 modules */
#include "candle_ros2/pds_modules/brake_resistor_ros.hpp"
#include "candle_ros2/pds_modules/isolated_converter_ros.hpp"
#include "candle_ros2/pds_modules/power_stage_ros.hpp"

/* CANdle-SDK */
#include "candle.hpp"
#include "pds.hpp"

class PdsNode : public rclcpp::Node
{
  public:
    PdsNode();
    ~PdsNode();

  private:
    std::unique_ptr<mab::Candle> candle;
    std::vector<PdsInstance>     pds_list;

    static constexpr const char* NODE_PREFIX  = "pds/";
    static constexpr int         PUB_TIMER_MS = 100;

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr srvAddPds;

    rclcpp::TimerBase::SharedPtr tmrPub;

    void cbAddPds(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                  std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp);

    std::unique_ptr<BaseModuleRos> createModule(mab::moduleType_E type);
};
