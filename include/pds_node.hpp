#pragma once
#include "rclcpp/rclcpp.hpp"

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

    std::string topicPrefix = "pds/";
};
