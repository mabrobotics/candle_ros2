#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include "candle_ros2/utils/candle_factory.hpp"
#include "candle_ros2/utils/candle_params.hpp"
#include "candle_ros2/md_node.hpp"
#include "candle_ros2/pds_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto           config_node = std::make_shared<rclcpp::Node>("candle_config");
    candleParams_S params      = readParams(config_node);
    auto           candle      = createCandle(params);

    auto md_node  = std::make_shared<MdNode>(rclcpp::NodeOptions(), candle, params);
    auto pds_node = std::make_shared<PdsNode>(rclcpp::NodeOptions(), candle, params);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(md_node);
    exec.add_node(pds_node);
    exec.add_node(config_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
