#include "md_node.hpp"

MdNode::MdNode() : Node("candle_ros_node")
{
    this->declare_parameter<std::string>("baud", "1M");
    this->declare_parameter<std::string>("bus", "USB");

    auto baud = mab::CANdleBaudrate_E::CAN_BAUD_1M;
    auto bus  = mab::candleTypes::busTypes_t::USB;

    std::string paramBaud = this->get_parameter("baud").as_string();
    std::string paramBus  = this->get_parameter("bus").as_string();

    if (paramBaud == "1M")
        baud = mab::CANdleBaudrate_E::CAN_BAUD_1M;
    else if (paramBaud == "2M")
        baud = mab::CANdleBaudrate_E::CAN_BAUD_2M;
    else if (paramBaud == "5M")
        baud = mab::CANdleBaudrate_E::CAN_BAUD_5M;
    else if (paramBaud == "8M")
        baud = mab::CANdleBaudrate_E::CAN_BAUD_8M;
    else
    {
        RCLCPP_INFO(this->get_logger(), "<baud> parameter not recognised!");
        return;
    }

    if (paramBus == "SPI")
        bus = mab::candleTypes::busTypes_t::SPI;
    else if (paramBus == "USB")
        bus = mab::candleTypes::busTypes_t::USB;
    else
    {
        RCLCPP_INFO(this->get_logger(), "<bus> parameter not recognised!");
        return;
    }

    candle = std::unique_ptr<mab::Candle>(mab::attachCandle(baud, bus));

    pubJointState =
        this->create_publisher<sensor_msgs::msg::JointState>(topicPrefix + "joint_states", 10);

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

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MdNode::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node has started.");
}

MdNode::~MdNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 node finished.");
}

void MdNode::publishJointStates()
{
    sensor_msgs::msg::JointState msgJointStates;

    msgJointStates.header.stamp = this->get_clock()->now();
    for (auto& md : mds)
    {
        msgJointStates.name.push_back(std::string("Joint " + std::to_string(md.m_canId)));
        msgJointStates.position.push_back(md.getPosition().first);
        msgJointStates.velocity.push_back(md.getVelocity().first);
        msgJointStates.effort.push_back(md.getTorque().first);
    }
    this->pubJointState->publish(msgJointStates);
    return;
}

void MdNode::cbMotionCmd(const candle_ros::msg::MotionCmd& msg)
{
    return;
}

void MdNode::cbPositionCmd(const candle_ros::msg::PositionPidCmd& msg)
{
    return;
}

void MdNode::cbVelocityCmd(const candle_ros::msg::VelocityPidCmd& msg)
{
    return;
}

void MdNode::cbImpedanceCmd(const candle_ros::msg::ImpedanceCmd& msg)
{
    return;
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros::srv::AddMds::Request> req,
                     std::shared_ptr<candle_ros::srv::AddMds::Response>      rsp)
{
    for (auto id : req->drive_ids)
    {
        mab::MD md(id, candle.get());
        if (md.init() == mab::MD::Error_t::OK)
        {
            mds.push_back(std::move(md));
            rsp->drives_success.push_back(true);
        }
        else
            rsp->drives_success.push_back(false);
    }
    rsp->total_number_of_drives = static_cast<u16>(mds.size());
    return;
}

void MdNode::cbZero(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    for (auto id : req->drive_ids)
    {
        auto it = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& md) { return md.m_canId == id; });

        if (it != mds.end())
        {
            if (it->zero() == mab::MD::Error_t::OK)
                rsp->drives_success.push_back(true);
            else
                rsp->drives_success.push_back(false);
        }
        else
            rsp->drives_success.push_back(false);
    }
    return;
}

void MdNode::cbSetMode(const std::shared_ptr<candle_ros::srv::SetMode::Request> req,
                       std::shared_ptr<candle_ros::srv::SetMode::Response>      rsp)
{
    if (req->drive_ids.size() != req->mode.size())
    {
        rsp->drives_success.assign(req->drive_ids.size(), false);

        RCLCPP_WARN(this->get_logger(),
                    "SetMode request incomplete. Sizes of arrays do not match!");
        return;
    }

    rsp->drives_success.reserve(req->drive_ids.size());

    for (size_t i = 0; i < req->drive_ids.size(); i++)
    {
        mab::MdMode_E mode = mab::MdMode_E::IDLE;
        const auto&   m    = req->mode[i];

        if (m == "IMPEDANCE")
            mode = mab::MdMode_E::IMPEDANCE;
        else if (m == "POSITION_PID")
            mode = mab::MdMode_E::POSITION_PID;
        else if (m == "VELOCITY_PID")
            mode = mab::MdMode_E::VELOCITY_PID;
        else if (m == "RAW_TORQUE")
            mode = mab::MdMode_E::RAW_TORQUE;
        else
            mode = mab::MdMode_E::IDLE;
        RCLCPP_WARN(this->get_logger(),
                    "MODE %s not recognized, setting IDLE for drive with ID: %d",
                    req->mode[i].c_str(),
                    req->drive_ids[i]);
        mode = mab::MdMode_E::IDLE;

        auto it =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = req->drive_ids[i]](const mab::MD& md) { return md.m_canId == id; });

        if (it != mds.end())
        {
            if (it->setMotionMode(mode) == mab::MD::Error_t::OK)
                rsp->drives_success.push_back(true);
            else
                rsp->drives_success.push_back(false);
        }
        else
            rsp->drives_success.push_back(false);
    }
    return;
}

void MdNode::cbEnable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                      std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    for (auto id : req->drive_ids)
    {
        auto it = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& md) { return md.m_canId == id; });

        if (it != mds.end())
        {
            if (it->enable() == mab::MD::Error_t::OK)
                rsp->drives_success.push_back(true);
            else
                rsp->drives_success.push_back(false);
        }
        else
            rsp->drives_success.push_back(false);
    }
    return;
}

void MdNode::cbDisable(const std::shared_ptr<candle_ros::srv::Generic::Request> req,
                       std::shared_ptr<candle_ros::srv::Generic::Response>      rsp)
{
    for (auto id : req->drive_ids)
    {
        auto it = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& md) { return md.m_canId == id; });

        if (it != mds.end())
        {
            if (it->disable() == mab::MD::Error_t::OK)
                rsp->drives_success.push_back(true);
            else
                rsp->drives_success.push_back(false);
        }
        else
            rsp->drives_success.push_back(false);
    }
    return;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MdNode>());
    rclcpp::shutdown();
    return 0;
}
