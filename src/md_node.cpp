#include "candle_ros2/md_node.hpp"

MdNode::MdNode(const std::string&           ns,
               const rclcpp::NodeOptions&   options,
               std::shared_ptr<mab::Candle> candle,
               const candleParams_S&        params)
    : Node("candle_md_node", ns, options), m_candle(candle)
{
    rclcpp::QoS defaultQoS(10);
    defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        defaultQoS.best_effort();

    pubJointState =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states", defaultQoS);

    subMotionCmd = this->create_subscription<candle_ros2::msg::MotionCmd>(
        "motion_command", 10, std::bind(&MdNode::cbMotionCmd, this, std::placeholders::_1));
    subPositionCmd = this->create_subscription<candle_ros2::msg::PositionPidCmd>(
        "position_command", 10, std::bind(&MdNode::cbPositionCmd, this, std::placeholders::_1));
    subVelocityCmd = this->create_subscription<candle_ros2::msg::VelocityPidCmd>(
        "velocity_command", 10, std::bind(&MdNode::cbVelocityCmd, this, std::placeholders::_1));
    subImpedanceCmd = this->create_subscription<candle_ros2::msg::ImpedanceCmd>(
        "impedance_command", 10, std::bind(&MdNode::cbImpedanceCmd, this, std::placeholders::_1));

    srvAddMd = this->create_service<candle_ros2::srv::AddDevices>(
        "add_mds", std::bind(&MdNode::cbAddMd, this, std::placeholders::_1, std::placeholders::_2));
    srvZero = this->create_service<candle_ros2::srv::Generic>(
        "zero", std::bind(&MdNode::cbZero, this, std::placeholders::_1, std::placeholders::_2));
    srvSetMode = this->create_service<candle_ros2::srv::SetMode>(
        "set_mode",
        std::bind(&MdNode::cbSetMode, this, std::placeholders::_1, std::placeholders::_2));
    srvEnable = this->create_service<candle_ros2::srv::Generic>(
        "enable", std::bind(&MdNode::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = this->create_service<candle_ros2::srv::Generic>(
        "disable",
        std::bind(&MdNode::cbDisable, this, std::placeholders::_1, std::placeholders::_2));

    tmrLoop = this->create_wall_timer(std::chrono::milliseconds(LOOP_TIMER_MS),
                                      std::bind(&MdNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node started.");
}

MdNode::~MdNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::controlLoop()
{
    sensor_msgs::msg::JointState msgJointStates;

    msgJointStates.name.reserve(m_mds.size());
    msgJointStates.position.reserve(m_mds.size());
    msgJointStates.velocity.reserve(m_mds.size());
    msgJointStates.effort.reserve(m_mds.size());

    mab::MDRegisters_S regs;

    msgJointStates.header.stamp = this->get_clock()->now();
    for (size_t i = 0; i < m_mds.size(); i++)
    {
        mab::MD& md  = m_mds[i];
        mdCmd_S& cmd = m_cmds[i];

        if (cmd.pending)
        {
            cmd.pending         = false;
            regs.targetPosition = cmd.position;
            regs.targetVelocity = cmd.velocity;
            regs.targetTorque   = cmd.torque;
            if (md.writeRegisters(regs.targetPosition, regs.targetVelocity, regs.targetTorque) !=
                mab::MD::Error_t::OK)
                RCLCPP_WARN_THROTTLE(this->get_logger(),
                                     *this->get_clock(),
                                     1000,
                                     "Failed to set Motion Command for drive with ID: %d",
                                     md.m_canId);
        }

        if (md.readRegisters(regs.mainEncoderPosition,
                             regs.mainEncoderVelocity,
                             regs.motorTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 1000,
                                 "Failed to read state of drive with ID: %d",
                                 md.m_canId);
            continue;
        }

        msgJointStates.name.push_back(std::string("Joint " + std::to_string(md.m_canId)));
        msgJointStates.position.push_back(regs.mainEncoderPosition.value);
        msgJointStates.velocity.push_back(regs.mainEncoderVelocity.value);
        msgJointStates.effort.push_back(regs.motorTorque.value);
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
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        m_cmds[md - m_mds.begin()] = {
            msg.target_position[i], msg.target_velocity[i], msg.target_torque[i], true};
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
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorPosPidKp     = msg.position_pid[i].kp;
        mdRegisters.motorPosPidKi     = msg.position_pid[i].ki;
        mdRegisters.motorPosPidKd     = msg.position_pid[i].kd;
        mdRegisters.motorPosPidWindup = msg.position_pid[i].i_windup;
        mdRegisters.maxVelocity       = msg.position_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorPosPidKp,
                               mdRegisters.motorPosPidKi,
                               mdRegisters.motorPosPidKd,
                               mdRegisters.motorPosPidWindup,
                               mdRegisters.maxVelocity) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Position PID parameters for drive with ID: %d",
                        msg.device_ids[i]);
        }

        if (i < (size_t)msg.velocity_pid.size())
        {
            mdRegisters.motorVelPidKp     = msg.velocity_pid[i].kp;
            mdRegisters.motorVelPidKi     = msg.velocity_pid[i].ki;
            mdRegisters.motorVelPidKd     = msg.velocity_pid[i].kd;
            mdRegisters.motorVelPidWindup = msg.velocity_pid[i].i_windup;
            mdRegisters.maxTorque         = msg.velocity_pid[i].max_output;
            if (md->writeRegisters(mdRegisters.motorVelPidKp,
                                   mdRegisters.motorVelPidKi,
                                   mdRegisters.motorVelPidKd,
                                   mdRegisters.motorVelPidWindup,
                                   mdRegisters.maxTorque) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Velocity PID parameters for drive with ID: %d",
                            msg.device_ids[i]);
            }
        }
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
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorVelPidKp     = msg.velocity_pid[i].kp;
        mdRegisters.motorVelPidKi     = msg.velocity_pid[i].ki;
        mdRegisters.motorVelPidKd     = msg.velocity_pid[i].kd;
        mdRegisters.motorVelPidWindup = msg.velocity_pid[i].i_windup;
        mdRegisters.maxTorque         = msg.velocity_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorVelPidKp,
                               mdRegisters.motorVelPidKi,
                               mdRegisters.motorVelPidKd,
                               mdRegisters.motorVelPidWindup,
                               mdRegisters.maxTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Velocity PID parameters for drive with ID: %d",
                        msg.device_ids[i]);
        }
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
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorImpPidKp = msg.kp[i];
        mdRegisters.motorImpPidKd = msg.kd[i];
        mdRegisters.maxTorque     = msg.max_output[i];
        if (md->writeRegisters(mdRegisters.motorImpPidKp,
                               mdRegisters.motorImpPidKd,
                               mdRegisters.maxTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Impedance parameters for drive with ID: %d",
                        msg.device_ids[i]);
        }
    }
    return;
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                     std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        if (findMd(m_mds, id) != m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is already added!", id);
            rsp->success.push_back(false);
            continue;
        }

        mab::MD md(id, m_candle.get());
        if (md.init() != mab::MD::Error_t::OK)
        {
            rsp->success.push_back(false);
            continue;
        }

        m_mds.push_back(std::move(md));
        m_cmds.push_back({});
        rsp->success.push_back(true);
    }
    rsp->total_devices = static_cast<u16>(m_mds.size());
    return;
}

void MdNode::cbZero(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (md->zero() == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
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
        bool               known   = true;

        if (reqMode == "IDLE")
            mode = mab::MdMode_E::IDLE;
        else if (reqMode == "IMPEDANCE")
            mode = mab::MdMode_E::IMPEDANCE;
        else if (reqMode == "POSITION_PID")
            mode = mab::MdMode_E::POSITION_PID;
        else if (reqMode == "VELOCITY_PID")
            mode = mab::MdMode_E::VELOCITY_PID;
        else if (reqMode == "RAW_TORQUE")
            mode = mab::MdMode_E::RAW_TORQUE;
        else
        {
            known = false;
            RCLCPP_WARN(this->get_logger(),
                        "MODE %s not recognized, setting IDLE for drive with ID: %d",
                        reqMode.c_str(),
                        req->device_ids[i]);
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (md->setMotionMode(mode) == mab::MD::Error_t::OK && known)
            rsp->success.push_back(true);
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
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (md->enable() == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
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
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (md->disable() == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
        else
            rsp->success.push_back(false);
    }
    return;
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
