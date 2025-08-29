#include "md_node.hpp"

MdNode::MdNode() : Node("candle_ros_node")
{
    this->declare_parameter<int>("baud", "1M");
    this->declare_parameter<std::string>("bus", "USB");

    auto baud = mab::CANdleBaudrate_E::CAN_BAUD_1M;
    auto bus  = mab::candleTypes::busTypes_t::USB;

    std::string paramBaud = this->get_parameter("baud").as_string();
    std::string paramBus  = this->get_parameter("bus").as_string();

    if (strcmp(paramBaud, "1M") == 0)
        baud = mab::CANdleBaudrate_E::CAN_BAUD_1M;
    else if (strcmp(paramBaud, "2M") == 0)
        baud = mab::CANdleBaudrate_E::CAN_BAUD_2M;
    else if (strcmp(paramBaud, "5M") == 0)
        baud = mab::CANdleBaudrate_E::CAN_BAUD_5M;
    else if (strcmp(paramBaud, "8M") == 0)
        baud = mab::CANdleBaudrate_E::CAN_BAUD_8M;
    else
    {
        RCLCPP_INFO(this->get_logger(), "<baud> parameter not recognised!");
        return;
    }

    if (strcmp(paramBus, "SPI") == 0)
        bus = mmab::candleTypes::busTypes_t::SPI;
    else if (strcmp(paramBus, "USB") == 0)
        bus = mab::candleTypes::busTypes_t::USB;
    else
    {
        RCLCPP_INFO(this->get_logger(), "<bus> parameter not recognised!");
        return;
    }

    candle = std::unique_ptr<mab::Candle>(mab::attachCandle(baud, bus));

    pubJointState =
        this->create_publisher<sensor_msgs::msg::JointState>(topicPrefix + "joint_states", qosRT);

    subMotionCmd = this->create_subscription<candle_ros::msg::MotionCmd>(
        topicPrefix + "motion_command",
        10,
        std::bind(&MdNode::cbMotionCmd, this, std::placeholders::_1));
    subPositionCmd = this->create_subscription<candle_ros::msg::PositionPidCmd>(
        topicPrefix + "position_command",
        10,
        std::bind(&MdNode::cbPositionCmd, this, std::placeholders::_1));
    subVelocityCmd = this->create_subscription<candle_ros::msg::VelocityPidCmd>(
        topicPrefix + "velocity_command",
        10,
        std::bind(&MdNode::cbVelocityCmd, this, std::placeholders::_1));
    subImpedanceCmd = this->create_subscription<candle_ros::msg::ImpedanceCmd>(
        topicPrefix + "impedance_command",
        10,
        std::bind(&MdNode::cbImpedanceCmd, this, std::placeholders::_1));

    srvAddMd = this->create_service<candle_ros::srv::AddMds>(
        topicPrefix + "add_mds",
        std::bind(&MdNode::cbAddMd, this, std::placeholders::_1, std::placeholders::_2));
    srvZero = this->create_service<candle_ros::srv::Generic>(
        topicPrefix + "zero",
        std::bind(&MdNode::cbZero, this, std::placeholders::_1, std::placeholders::_2));
    srvSetMode = this->create_service<candle_ros::srv::SetMode>(
        topicPrefix + "set_mode",
        std::bind(&MdNode::cbSetMode, this, std::placeholders::_1, std::placeholders::_2));
    srvEnable = this->create_service<candle_ros::srv::Generic>(
        topicPrefix + "enable",
        std::bind(&MdNode::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = this->create_service<candle_ros::srv::Generic>(
        topicPrefix + "disable",
        std::bind(&MdNode::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

    pubTimer = n.createTimer(ros::Duration(0.1), std::bind(&Md80Node::publishJointStates, this));
    pubTimer.stop();

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node has started.");
}

Md80Node::~MdNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node finished.");
}

void Md80Node::publishJointStates()
{
    sensor_msgs::JointState jointStateMsg;
    jointStateMsg.header.stamp = ros::Time::now();
    for (auto candle : candleInstances)
    {
        for (auto& md : candle->md80s)
        {
            jointStateMsg.name.push_back(std::string("Joint " + std::to_string(md.getId())));
            jointStateMsg.position.push_back(md.getPosition());
            jointStateMsg.velocity.push_back(md.getVelocity());
            jointStateMsg.effort.push_back(md.getTorque());
        }
    }

    this->jointStatePub.publish(jointStateMsg);
}

void cbMotionCmd(const candle_ros::msg::MotionCmd& msg)
{
    return;
}

void cbPositionCmd(const candle_ros::msg::PositionPidCmd& msg)
{
    return;
}

void cbVelocityCmd(const candle_ros::msg::VelocityPidCmd& msg)
{
    return;
}

void cbImpedanceCmd(const candle_ros::msg::ImpedanceCmd& msg)
{
    return;
}

void cbAddMd(const std::shared_ptr<candle_ros::srv::AddMds::Request> req,
             std::shared_ptr<candle_ros::srv::AddMds::Response>      rsp)
{
    return;
}

void cbZero(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
            std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    return;
}

void cbSetMode(const std::shared_ptr<candle_ros::srv::SetMode::Request> req,
               std::shared_ptr<candle_ros::srv::SetMode::Response>      rsp)
{
    return;
}

void cbEnable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
              std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    return;
}

void cbDisable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
               std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    return;
}

// mab::Candle* Md80Node::findCandleByMd80Id(uint16_t md80Id)
// {
//     for (auto candle : candleInstances)
//     {
//         for (auto id : candle->md80s)
//         {
//             if (id.getId() == md80Id)
//                 return candle;
//         }
//     }
//     return NULL;
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "candle_ros_node");
    Md80Node n(argc, argv);
    ros::spin();

    return 0;
}
