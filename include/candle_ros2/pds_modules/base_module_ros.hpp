#pragma once
#include "rclcpp/rclcpp.hpp"

/* CANdle-SDK */
#include "pds.hpp"

class I_BaseModuleRos
{
  public:
    virtual ~I_BaseModuleRos() = default;

    virtual bool setup(std::shared_ptr<rclcpp::Node> node,
                       mab::Pds&                     pds,
                       mab::socketIndex_E            socket,
                       const int                     pdsId,
                       rclcpp::QoS&                  qos,
                       const std::string&            nodePrefix = "pds/",
                       const int                     timerMs    = 1000) = 0;

  protected:
    std::shared_ptr<rclcpp::Node> m_parentNode;

    rclcpp::TimerBase::SharedPtr tmrPub;
};
