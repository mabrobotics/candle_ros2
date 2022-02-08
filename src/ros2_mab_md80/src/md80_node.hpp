#include "candle.hpp"

#include "ros2_mab_md80/msg/impedance_command.hpp"
#include "ros2_mab_md80/srv/add_md80s.hpp"
#include "ros2_mab_md80/srv/zero_md80s.hpp"
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
    ros2_mab_md80::msg::ImpedanceCommand imp;
    rclcpp::Service<ros2_mab_md80::srv::AddMd80s>::SharedPtr addMd80Service;

    void service_addMd80(const std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Response> response);
    void service_zeroMd80(const std::shared_ptr<ros2_mab_md80::srv::ZeroMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::ZeroMd80s::Response> response);
    void service_setModeMd80(const std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Response> response);
};