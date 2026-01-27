#include "rclcpp/rclcpp.hpp"

/* Candle ROS2 Nodes */
#include "candle_ros2/md_node.hpp"
#include "candle_ros2/pds_node.hpp"

/* Utils */
#include "candle_ros2/utils/candle_factory.hpp"
#include "candle_ros2/utils/candle_params.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto config_node = std::make_shared<rclcpp::Node>("candle_config");

    config_node->declare_parameter<bool>("launch_md_node", true);
    config_node->declare_parameter<bool>("launch_pds_node", true);

    bool launch_md  = config_node->get_parameter("launch_md_node").as_bool();
    bool launch_pds = config_node->get_parameter("launch_pds_node").as_bool();

    candleParams_S params = readParams(config_node);
    auto           candle = createCandle(params);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(config_node);

    rclcpp::Node::SharedPtr md_node;
    rclcpp::Node::SharedPtr pds_node;

    if (launch_md)
    {
        md_node = std::make_shared<MdNode>(rclcpp::NodeOptions(), candle, params);
        exec.add_node(md_node);
    }

    if (launch_pds)
    {
        pds_node = std::make_shared<PdsNode>(rclcpp::NodeOptions(), candle, params);
        exec.add_node(pds_node);
    }

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
