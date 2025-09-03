#pragma once
#include "rclcpp/rclcpp.hpp"

#include "pds.hpp"

class BaseModuleRos
{
  public:
    virtual ~BaseModuleRos() = default;

    virtual bool setup(std::shared_ptr<rclcpp::Node> node,
                       mab::Pds&                     pds,
                       mab::socketIndex_E            socket,
                       const int                     pdsId,
                       const std::string&            nodePrefix = "pds/",
                       const int                     timerMs    = 1000) = 0;

  protected:
    std::shared_ptr<rclcpp::Node> parentNode;

    rclcpp::TimerBase::SharedPtr tmrPub;
};
