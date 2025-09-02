#pragma once
#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros/msg/impedance_cmd.hpp"
#include "candle_ros/msg/motion_cmd.hpp"
#include "candle_ros/msg/position_pid_cmd.hpp"
#include "candle_ros/msg/velocity_pid_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/* Services */
#include "candle_ros/srv/add_devices.hpp"
#include "candle_ros/srv/generic.hpp"
#include "candle_ros/srv/set_limits.hpp"
#include "candle_ros/srv/set_mode.hpp"

/* CANdle-SDK */
#include "candle.hpp"
#include "MD.hpp"

class MdNode : public rclcpp::Node
{
  public:
    MdNode();
    ~MdNode();

  private:
    std::unique_ptr<mab::Candle> candle;
    std::vector<mab::MD>         mds;

    std::string topicPrefix = "md/";

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointState;

    rclcpp::Subscription<candle_ros::msg::MotionCmd>::SharedPtr      subMotionCmd;
    rclcpp::Subscription<candle_ros::msg::PositionPidCmd>::SharedPtr subPositionCmd;
    rclcpp::Subscription<candle_ros::msg::VelocityPidCmd>::SharedPtr subVelocityCmd;
    rclcpp::Subscription<candle_ros::msg::ImpedanceCmd>::SharedPtr   subImpedanceCmd;

    rclcpp::Service<candle_ros::srv::AddDevices>::SharedPtr srvAddMd;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr    srvZero;
    rclcpp::Service<candle_ros::srv::SetMode>::SharedPtr    srvSetMode;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr    srvEnable;
    rclcpp::Service<candle_ros::srv::Generic>::SharedPtr    srvDisable;

    rclcpp::TimerBase::SharedPtr tmrPub;

    void publishJointStates();

    void cbMotionCmd(const candle_ros::msg::MotionCmd& msg);
    void cbPositionCmd(const candle_ros::msg::PositionPidCmd& msg);
    void cbVelocityCmd(const candle_ros::msg::VelocityPidCmd& msg);
    void cbImpedanceCmd(const candle_ros::msg::ImpedanceCmd& msg);

    void cbAddMd(const std::shared_ptr<candle_ros::srv::AddDevices::Request> req,
                 std::shared_ptr<candle_ros::srv::AddDevices::Response>      rsp);
    void cbZero(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);
    void cbSetMode(const std::shared_ptr<candle_ros::srv::SetMode::Request> req,
                   std::shared_ptr<candle_ros::srv::SetMode::Response>      rsp);
    void cbEnable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                  std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);
    void cbDisable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                   std::shared_ptr<candle_ros::srv::Generic::Response>      rsp);
};
