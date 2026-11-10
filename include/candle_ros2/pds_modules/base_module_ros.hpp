#pragma once
#include "rclcpp/rclcpp.hpp"

/* Services */
#include "candle_ros2/srv/generic_pds.hpp"

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

    rclcpp::Service<candle_ros2::srv::GenericPds>::SharedPtr srvEnable;
    rclcpp::Service<candle_ros2::srv::GenericPds>::SharedPtr srvDisable;

    static std::string topicName(const int          pdsId,
                                 const std::string& name,
                                 mab::socketIndex_E socket)
    {
        return "id_" + std::to_string(pdsId) + "/" + name + "_" +
               std::to_string(static_cast<int>(socket));
    }

    template <class T>
    void createEnableServices(T*                 module,
                              const int          pdsId,
                              const char*        name,
                              mab::socketIndex_E socket)
    {
        using Srv = candle_ros2::srv::GenericPds;

        srvEnable = m_parentNode->create_service<Srv>(
            topicName(pdsId, std::string("enable_") + name, socket),
            [module](const std::shared_ptr<Srv::Request>, std::shared_ptr<Srv::Response> rsp)
            { rsp->success.push_back(module->enable() == mab::PdsModule::error_E::OK); });

        srvDisable = m_parentNode->create_service<Srv>(
            topicName(pdsId, std::string("disable_") + name, socket),
            [module](const std::shared_ptr<Srv::Request>, std::shared_ptr<Srv::Response> rsp)
            { rsp->success.push_back(module->disable() == mab::PdsModule::error_E::OK); });
    }
};
