#include "candle.hpp"

#include "ros2_mab_md80/msg/impedance_command.hpp"
#include "ros2_mab_md80/srv/add_md80s.hpp"
#include "ros2_mab_md80/srv/generic_md80_msg.hpp"
#include "ros2_mab_md80/srv/set_mode_md80s.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class Md80Node : public rclcpp::Node
{
public:
    Md80Node();
    ~Md80Node();

private:
    mab::Candle*candle;

    rclcpp::TimerBase::SharedPtr pubTimer;
    
    rclcpp::Service<ros2_mab_md80::srv::AddMd80s>::SharedPtr addMd80Service;
    rclcpp::Service<ros2_mab_md80::srv::GenericMd80Msg>::SharedPtr zeroMd80Service;
    rclcpp::Service<ros2_mab_md80::srv::SetModeMd80s>::SharedPtr setModeMd80Service;
    rclcpp::Service<ros2_mab_md80::srv::GenericMd80Msg>::SharedPtr enableMd80Service;
    rclcpp::Service<ros2_mab_md80::srv::GenericMd80Msg>::SharedPtr disableMd80Service;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    
    void service_addMd80(const std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Response> response);
    void service_zeroMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response);
    void service_setModeMd80(const std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Response> response);
    void service_enableMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response);
    void service_disableMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response);
    void publishJointStates();
};
