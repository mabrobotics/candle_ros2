#pragma once
#include "rclcpp/rclcpp.hpp"

/* CANdle-SDK */
#include "pds.hpp"

class I_BaseModuleRos
{
  public:
    virtual ~I_BaseModuleRos() = default;

    virtual bool setup(rclcpp::Node*      node,
                       mab::Pds&          pds,
                       mab::socketIndex_E socket,
                       const int          pdsId,
                       rclcpp::QoS&       qos,
                       const int          timerMs = 1000) = 0;

  protected:
    rclcpp::Node* m_parentNode{nullptr};

    rclcpp::TimerBase::SharedPtr tmrPub;
};
