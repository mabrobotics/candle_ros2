#include "candle_ros2/md_node.hpp"

MdNode::MdNode() : Node("candle_md_node")
{
    this->declare_parameter<std::string>("data_rate", "1M");
    this->declare_parameter<std::string>("bus", "USB");

    auto dataRate = mab::CANdleDatarate_E::CAN_DATARATE_1M;
    auto bus      = mab::candleTypes::busTypes_t::USB;

    std::string paramDataRate = this->get_parameter("data_rate").as_string();
    std::string paramBus      = this->get_parameter("bus").as_string();

    if (paramDataRate == "1M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_1M;
    else if (paramDataRate == "2M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_2M;
    else if (paramDataRate == "5M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_5M;
    else if (paramDataRate == "8M")
        dataRate = mab::CANdleDatarate_E::CAN_DATARATE_8M;
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "<data_rate> parameter not recognized! Value: '%s'",
                    paramDataRate.c_str());
        return;
    }

    if (paramBus == "SPI")
        bus = mab::candleTypes::busTypes_t::SPI;
    else if (paramBus == "USB")
        bus = mab::candleTypes::busTypes_t::USB;
    else
    {
        RCLCPP_INFO(
            this->get_logger(), "<bus> parameter not recognized! Value: %s", paramBus.c_str());
        return;
    }

    candle = std::unique_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));

    pubJointState = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(NODE_PREFIX) + "joint_states", 10);

    subMotionCmd = this->create_subscription<candle_ros2::msg::MotionCmd>(
        std::string(NODE_PREFIX) + "motion_command",
        10,
        std::bind(&MdNode::cbMotionCmd, this, std::placeholders::_1));
    subPositionCmd = this->create_subscription<candle_ros2::msg::PositionPidCmd>(
        std::string(NODE_PREFIX) + "position_command",
        10,
        std::bind(&MdNode::cbPositionCmd, this, std::placeholders::_1));
    subVelocityCmd = this->create_subscription<candle_ros2::msg::VelocityPidCmd>(
        std::string(NODE_PREFIX) + "velocity_command",
        10,
        std::bind(&MdNode::cbVelocityCmd, this, std::placeholders::_1));
    subImpedanceCmd = this->create_subscription<candle_ros2::msg::ImpedanceCmd>(
        std::string(NODE_PREFIX) + "impedance_command",
        10,
        std::bind(&MdNode::cbImpedanceCmd, this, std::placeholders::_1));

    srvAddMd = this->create_service<candle_ros2::srv::AddDevices>(
        std::string(NODE_PREFIX) + "add_mds",
        std::bind(&MdNode::cbAddMd, this, std::placeholders::_1, std::placeholders::_2));
    srvZero = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "zero",
        std::bind(&MdNode::cbZero, this, std::placeholders::_1, std::placeholders::_2));
    srvSetMode = this->create_service<candle_ros2::srv::SetMode>(
        std::string(NODE_PREFIX) + "set_mode",
        std::bind(&MdNode::cbSetMode, this, std::placeholders::_1, std::placeholders::_2));
    srvEnable = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "enable",
        std::bind(&MdNode::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "disable",
        std::bind(&MdNode::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MdNode::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node started.");
}

MdNode::~MdNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
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

void MdNode::cbMotionCmd(const candle_ros2::msg::MotionCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.target_position.size() || n != msg.target_velocity.size() ||
        n != msg.target_torque.size())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Motion Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = msg.device_ids[i]](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->setTargetPosition(msg.target_position[i]) != mab::MD::Error_t::OK)
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Target Position for drive with ID: %d",
                            msg.device_ids[i]);
            if (md->setTargetVelocity(msg.target_velocity[i]) != mab::MD::Error_t::OK)
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Target Velocity for drive with ID: %d",
                            msg.device_ids[i]);
            if (md->setTargetTorque(msg.target_torque[i]) != mab::MD::Error_t::OK)
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Target Torque for drive with ID: %d",
                            msg.device_ids[i]);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
    }
    return;
}

void MdNode::cbPositionCmd(const candle_ros2::msg::PositionPidCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.position_pid.size())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Position Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = msg.device_ids[i]](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->setPositionPIDparam(msg.position_pid[i].kp,
                                        msg.position_pid[i].ki,
                                        msg.position_pid[i].kd,
                                        msg.position_pid[i].i_windup) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Position PID parameters for drive with ID: %d",
                            msg.device_ids[i]);
            }
            if (md->setProfileVelocity(msg.position_pid[i].max_output) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Profile Velocity for drive with ID: %d",
                            msg.device_ids[i]);
            }

            if (i < (size_t)msg.velocity_pid.size())
            {
                if (md->setVelocityPIDparam(msg.velocity_pid[i].kp,
                                            msg.velocity_pid[i].ki,
                                            msg.velocity_pid[i].kd,
                                            msg.velocity_pid[i].i_windup) != mab::MD::Error_t::OK)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Failed to set Velocity PID parameters for drive with ID: %d",
                                msg.device_ids[i]);
                }
                if (md->setMaxTorque(msg.velocity_pid[i].max_output) != mab::MD::Error_t::OK)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Failed to set Max Torque for drive with ID: %d",
                                msg.device_ids[i]);
                }
            }
        }
        else
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
    }
    return;
}

void MdNode::cbVelocityCmd(const candle_ros2::msg::VelocityPidCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.velocity_pid.size())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Velocity Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = msg.device_ids[i]](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->setVelocityPIDparam(msg.velocity_pid[i].kp,
                                        msg.velocity_pid[i].ki,
                                        msg.velocity_pid[i].kd,
                                        msg.velocity_pid[i].i_windup) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Velocity PID parameters for drive with ID: %d",
                            msg.device_ids[i]);
            }
            if (md->setMaxTorque(msg.velocity_pid[i].max_output) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Max Torque for drive with ID: %d",
                            msg.device_ids[i]);
            }
        }
        else
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
    }
    return;
}

void MdNode::cbImpedanceCmd(const candle_ros2::msg::ImpedanceCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.kp.size() || n != msg.kd.size() || n != msg.max_output.size())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Impedance Command message incomplete. Sizes of arrays do not match! Ignoring "
                    "message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = msg.device_ids[i]](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->setImpedanceParams(msg.kp[i], msg.kd[i]) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Impedance parameters for drive with ID: %d",
                            msg.device_ids[i]);
            }
            if (md->setMaxTorque(msg.max_output[i]) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Max Torque for drive with ID: %d",
                            msg.device_ids[i]);
            }
        }
        else
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
    }
    return;
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                     std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        mab::MD md(id, candle.get());
        if (md.init() == mab::MD::Error_t::OK)
        {
            mds.push_back(std::move(md));
            rsp->success.push_back(true);
        }
        else
            rsp->success.push_back(false);
    }
    rsp->total_devices = static_cast<u16>(mds.size());
    return;
}

void MdNode::cbZero(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->zero() == mab::MD::Error_t::OK)
                rsp->success.push_back(true);
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }
    return;
}

void MdNode::cbSetMode(const std::shared_ptr<candle_ros2::srv::SetMode::Request> req,
                       std::shared_ptr<candle_ros2::srv::SetMode::Response>      rsp)
{
    if (req->device_ids.size() != req->mode.size())
    {
        rsp->success.assign(req->device_ids.size(), false);

        RCLCPP_WARN(this->get_logger(),
                    "SetMode request incomplete. Sizes of arrays do not match!");
        return;
    }

    rsp->success.reserve(req->device_ids.size());

    for (size_t i = 0; i < req->device_ids.size(); i++)
    {
        mab::MdMode_E      mode    = mab::MdMode_E::IDLE;
        const std::string& reqMode = req->mode[i];

        if (reqMode == "IMPEDANCE")
            mode = mab::MdMode_E::IMPEDANCE;
        else if (reqMode == "POSITION_PID")
            mode = mab::MdMode_E::POSITION_PID;
        else if (reqMode == "VELOCITY_PID")
            mode = mab::MdMode_E::VELOCITY_PID;
        else if (reqMode == "RAW_TORQUE")
            mode = mab::MdMode_E::RAW_TORQUE;
        else
        {
            mode = mab::MdMode_E::IDLE;
            RCLCPP_WARN(this->get_logger(),
                        "MODE %s not recognized, setting IDLE for drive with ID: %d",
                        reqMode.c_str(),
                        req->device_ids[i]);
        }

        auto md =
            std::find_if(mds.begin(),
                         mds.end(),
                         [id = req->device_ids[i]](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->setMotionMode(mode) == mab::MD::Error_t::OK)
                rsp->success.push_back(true);
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }
    return;
}

void MdNode::cbEnable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                      std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->enable() == mab::MD::Error_t::OK)
                rsp->success.push_back(true);
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }
    return;
}

void MdNode::cbDisable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                       std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = std::find_if(
            mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });

        if (md != mds.end())
        {
            if (md->disable() == mab::MD::Error_t::OK)
                rsp->success.push_back(true);
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
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
