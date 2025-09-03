#include "rclcpp/rclcpp.hpp"

#include "candle_ros2/msg/isolated_converter_data.hpp"

#include "candle_ros2/pds_modules/base_module_ros.hpp"

#include "pds.hpp"

class IsolatedConverterRos : public BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     pdsId,
               const std::string&            nodePrefix = "pds/",
               const int                     timerMs    = 1000) override;

  private:
    std::shared_ptr<mab::IsolatedConv> isolatedConverter;

    rclcpp::Publisher<candle_ros2::msg::IsolatedConverterData>::SharedPtr pubData;

    void publishStatus();
};