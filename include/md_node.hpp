#pragma once
#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros/msg/ImpedanceCmd.h"
#include "candle_ros/msg/MotionCmd.h"
#include "candle_ros/msg/PositionPidCmd.h"
#include "candle_ros/msg/VelocityPidCmd.h"

/* Services */
#include "candle_ros/srv/AddMds.h"
#include "candle_ros/srv/Generic.h"
#include "candle_ros/srv/SetLimits.h"
#include "candle_ros/srv/SetMode.h"
#include "sensor_msgs/JointState.h"

/* CANdle-SDK */
#include "candle.hpp"
#include "MD.hpp"

class MdNode : public rclcpp::Node
{
  public:
    MdNode();
    ~MdNode();

  private:
    std::string topicPrefix = "md/";

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointState;

    rclcpp::Subscription<candle_ros::msg::MotionCmd>::SharedPtr      subMotionCmd;
    rclcpp::Subscription<candle_ros::msg::PositionPidCmd>::SharedPtr subPositionCmd;
    rclcpp::Subscription<candle_ros::msg::VelocityPidCmd>::SharedPtr subVelocityCmd;
    rclcpp::Subscription<candle_ros::msg::ImpedanceCmd>::SharedPtr   subImpedanceCmd;

    rclcpp::Service<candle_ros::srv::AddMds>::SharedPtr  srvAddMd;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr srvZero;
    rclcpp::Service<candle_ros::srv::SetMode>::SharedPtr srvSetMode;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr srvEnable;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr srvDisable;

    std::vector<mab::Candle*> candleInstances;

    void publishJointStates();

    void cbMotionCmd(const candle_ros::msg::MotionCmd& msg);
    void cbPositionCmd(const candle_ros::msg::PositionPidCmd& msg);
    void cbVelocityCmd(const candle_ros::msg::VelocityPidCmd& msg);
    void cbImpedanceCmd(const candle_ros::msg::ImpedanceCmd& msg);

    void cbAddMd(const std::shared_ptr<candle_ros::srv::AddMds::Request> req,
                 std::shared_ptr<candle_ros::srv::AddMds::Response>      rsp);
    void cbZero(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);
    void cbSetMode(const std::shared_ptr<candle_ros::srv::SetMode::Request> req,
                   std::shared_ptr<candle_ros::srv::SetMode::Response>      rsp);
    void cbEnable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                  std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);
    void cbDisable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                   std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);

    // mab::Candle* findCandleByMd80Id(uint16_t md80Id);
};
