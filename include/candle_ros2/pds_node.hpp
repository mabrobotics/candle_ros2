#pragma once
#include "rclcpp/rclcpp.hpp"

/* Services */
#include "candle_ros2/srv/add_devices.hpp"
#include "candle_ros2/srv/generic.hpp"

/* CANdle-SDK */
#include "candle.hpp"
#include "pds.hpp"

#include "brake_resistor_ros.hpp"

class PdsNode : public rclcpp::Node
{
  public:
    PdsNode();
    ~PdsNode();

  private:
    std::unique_ptr<mab::Candle> candle;
    std::vector<mab::Pds>        pds_list;

    std::vector<std::unique_ptr<BrakeResistorRos>> br_list;

    std::string topicPrefix = "pds/";

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr srvAddPds;

    rclcpp::TimerBase::SharedPtr tmrPub;

    void cbAddPds(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                  std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp);
};
