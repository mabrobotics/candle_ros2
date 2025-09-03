#include "rclcpp/rclcpp.hpp"

#include "candle_ros2/msg/power_stage_data.hpp"

#include "candle_ros2/pds_modules/base_module_ros.hpp"

#include "pds.hpp"

class PowerStageRos : public BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     pdsId,
               const std::string&            nodePrefix = "pds/",
               const int                     timerMs    = 1000) override;

  private:
    std::shared_ptr<mab::PowerStage> powerStage;

    rclcpp::Publisher<candle_ros2::msg::PowerStageData>::SharedPtr pubData;

    void publishStatus();
};