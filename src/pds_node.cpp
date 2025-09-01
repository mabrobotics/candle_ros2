#include "pds_node.hpp"

PdsNode::PdsNode() : Node("candle_pds_node")
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node has started.");
}

PdsNode::~PdsNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node finished.");
}
