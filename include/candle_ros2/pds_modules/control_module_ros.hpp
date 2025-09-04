#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros2/msg/control_module_data.hpp"

/* PDS ROS2 modules */
#include "candle_ros2/pds_modules/base_module_ros.hpp"

/* CANdle-SDK */
#include "pds.hpp"

class ControlModuleRos : public BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     pdsId,
               const std::string&            nodePrefix = "pds/",
               const int                     timerMs    = 1000) override;

  private:
    static constexpr const char* MODULE_NAME = "control";

    mab::Pds* controlModule{nullptr};

    rclcpp::Publisher<candle_ros2::msg::ControlModuleData>::SharedPtr pubData;

    void publishStatus();
};
