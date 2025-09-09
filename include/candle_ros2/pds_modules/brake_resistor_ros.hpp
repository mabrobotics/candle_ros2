#pragma once
#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros2/msg/brake_resistor_data.hpp"

/* Services */
#include "candle_ros2/srv/generic_pds.hpp"

/* PDS ROS2 modules */
#include "candle_ros2/pds_modules/base_module_ros.hpp"

/* CANdle-SDK */
#include "pds.hpp"

class BrakeResistorRos : public I_BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     pdsId,
               rclcpp::QoS&                  qos,
               const std::string&            nodePrefix = "pds/",
               const int                     timerMs    = 1000) override;

  private:
    static constexpr const char* MODULE_NAME = "br";

    std::shared_ptr<mab::BrakeResistor> m_brakeResistor;

    rclcpp::Publisher<candle_ros2::msg::BrakeResistorData>::SharedPtr pubData;

    rclcpp::Service<candle_ros2::srv::GenericPds>::SharedPtr srvEnable;
    rclcpp::Service<candle_ros2::srv::GenericPds>::SharedPtr srvDisable;

    void publishStatus();

    void cbEnable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                  std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp);
    void cbDisable(const std::shared_ptr<candle_ros2::srv::GenericPds::Request> req,
                   std::shared_ptr<candle_ros2::srv::GenericPds::Response>      rsp);
};
