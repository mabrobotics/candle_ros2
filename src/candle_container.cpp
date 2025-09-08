#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#include "candle_ros2/utils/candle_factory.hpp"
#include "candle_ros2/md_node.hpp"
#include "candle_ros2/pds_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create helper node for parameter parsing
    auto           config_node = std::make_shared<rclcpp::Node>("candle_config");
    candleParams_S params      = readParams(config_node);
    auto           candle      = createCandle(params);

    // Container
    auto container = std::make_shared<rclcpp_components::ComponentManager>(rclcpp::NodeOptions{});

    // Load MdNode
    container->load_component("my_package",
                              "MdNode",
                              rclcpp::NodeOptions(),
                              [candle, params](const rclcpp::NodeOptions& options)
                              { return std::make_shared<MdNode>(options, candle, params); });

    // Load PdsNode
    container->load_component("my_package",
                              "PdsNode",
                              rclcpp::NodeOptions(),
                              [candle, params](const rclcpp::NodeOptions& options)
                              { return std::make_shared<PdsNode>(options, candle, params); });

    rclcpp::spin(container);
    rclcpp::shutdown();
    return 0;
}
