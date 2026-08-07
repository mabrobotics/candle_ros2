#include "candle_ros2/md_node.hpp"

#include <cmath>
#include <stdexcept>

MdNode::MdNode(const rclcpp::NodeOptions&   options,
               std::shared_ptr<mab::Candle> candle,
               const candleParams_S&        params)
    : Node("candle_md_node", options),
      m_candle(std::move(candle)),
      jointNamePrefix(params.joint_name_prefix),
      gripperOpenPositionRad(params.gripper_open_position_rad),
      gripperClosedPositionRad(params.gripper_closed_position_rad),
      gripperImpedanceKp(static_cast<float>(params.gripper_impedance_kp)),
      gripperImpedanceKd(static_cast<float>(params.gripper_impedance_kd)),
      gripperVelocityLimitRadS(static_cast<float>(params.gripper_velocity_limit_rad_s)),
      gripperTorqueLimitNm(static_cast<float>(params.gripper_torque_limit_nm)),
      initDevicesZero(params.init_devices_zero)
{
    if (jointNamePrefix.empty())
        throw std::invalid_argument("joint_name_prefix must not be empty");
    if (!std::isfinite(gripperOpenPositionRad) || !std::isfinite(gripperClosedPositionRad) ||
        !std::isfinite(gripperImpedanceKp) || gripperImpedanceKp < 0.0f ||
        !std::isfinite(gripperImpedanceKd) || gripperImpedanceKd < 0.0f ||
        !std::isfinite(gripperVelocityLimitRadS) || gripperVelocityLimitRadS <= 0.0f ||
        !std::isfinite(gripperTorqueLimitNm) || gripperTorqueLimitNm <= 0.0f)
        throw std::invalid_argument(
            "invalid gripper position, gain, velocity, or torque parameter");

    rclcpp::QoS defaultQoS(10);
    defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        defaultQoS.best_effort();

    pubJointState = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(NODE_PREFIX) + "joint_states", defaultQoS);

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
    srvInitDevices = this->create_service<candle_ros2::srv::InitDevices>(
        std::string(NODE_PREFIX) + "init_devices",
        std::bind(&MdNode::cbInitDevices, this, std::placeholders::_1, std::placeholders::_2));
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
    srvSetLimits = this->create_service<candle_ros2::srv::SetLimits>(
        std::string(NODE_PREFIX) + "set_limits",
        std::bind(&MdNode::cbSetLimits, this, std::placeholders::_1, std::placeholders::_2));
    srvOpen = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "open_gripper",
        std::bind(&MdNode::cbOpenGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvClose = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "close_gripper",
        std::bind(&MdNode::cbCloseGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvConfigureGripper = this->create_service<candle_ros2::srv::ConfigureGripper>(
        std::string(NODE_PREFIX) + "configure_gripper",
        std::bind(&MdNode::cbConfigureGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvSetGripperTargets = this->create_service<candle_ros2::srv::SetGripperTargets>(
        std::string(NODE_PREFIX) + "set_gripper_targets",
        std::bind(
            &MdNode::cbSetGripperTargets, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(PUB_TIMER_MS),
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

    msgJointStates.name.reserve(m_mds.size());
    msgJointStates.position.reserve(m_mds.size());
    msgJointStates.velocity.reserve(m_mds.size());
    msgJointStates.effort.reserve(m_mds.size());

    msgJointStates.header.stamp = this->get_clock()->now();
    for (auto& md : m_mds)
    {
        // One batched register read per drive is a third of the bus and USB
        // load of the per-register getters, and a failed read must never be
        // published as a measurement (the getters return a literal 0.0 on
        // error, which downstream consumers cannot tell from a real reading).
        if (md.readRegisters(md.m_mdRegisters.mainEncoderPosition,
                             md.m_mdRegisters.mainEncoderVelocity,
                             md.m_mdRegisters.motorTorque) != mab::MD::Error_t::OK)
            continue;
        msgJointStates.name.push_back(jointNamePrefix + std::to_string(md.m_canId));
        msgJointStates.position.push_back(md.m_mdRegisters.mainEncoderPosition.value);
        msgJointStates.velocity.push_back(md.m_mdRegisters.mainEncoderVelocity.value);
        msgJointStates.effort.push_back(md.m_mdRegisters.motorTorque.value);
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

        mab::MDRegisters_S mdRegisters;
        mdRegisters.targetPosition = msg.target_position[i];
        mdRegisters.targetVelocity = msg.target_velocity[i];
        mdRegisters.targetTorque   = msg.target_torque[i];

        if (md->writeRegisters(mdRegisters.targetPosition,
                               mdRegisters.targetVelocity,
                               mdRegisters.targetTorque) != mab::MD::Error_t::OK)
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Motion Command for drive with ID: %d",
                        msg.device_ids[i]);
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
        mdRegisters.motorVelPidKp     = msg.position_pid[i].kp;
        mdRegisters.motorVelPidKi     = msg.position_pid[i].ki;
        mdRegisters.motorVelPidKd     = msg.position_pid[i].kd;
        mdRegisters.motorVelPidWindup = msg.position_pid[i].i_windup;
        mdRegisters.profileVelocity   = msg.position_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorVelPidKp,
                               mdRegisters.motorVelPidKi,
                               mdRegisters.motorVelPidKd,
                               mdRegisters.motorVelPidWindup,
                               mdRegisters.profileVelocity) != mab::MD::Error_t::OK)
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
            rsp->success.push_back(true);
            continue;
        }

        mab::MD md(id, m_candle.get());
        if (md.init() != mab::MD::Error_t::OK)
        {
            rsp->success.push_back(false);
            continue;
        }

        // Give the drive 5 ms (50 x 100 us) to answer instead of the 1 ms
        // SDK default. On a loaded host the USB round trip alone can exceed
        // the default window, and the SDK misreports that as "CAN frame did
        // not reach target device" even though the bus is healthy.
        md.m_timeout = 50;

        m_mds.push_back(std::move(md));
        rsp->success.push_back(true);
    }
    rsp->total_devices = static_cast<u16>(m_mds.size());
    return;
}

void MdNode::cbInitDevices(const std::shared_ptr<candle_ros2::srv::InitDevices::Request> req,
                           std::shared_ptr<candle_ros2::srv::InitDevices::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);

    auto addReq        = std::make_shared<candle_ros2::srv::AddDevices::Request>();
    auto addRsp        = std::make_shared<candle_ros2::srv::AddDevices::Response>();
    addReq->device_ids = req->device_ids;
    cbAddMd(addReq, addRsp);

    auto modeReq        = std::make_shared<candle_ros2::srv::SetMode::Request>();
    auto modeRsp        = std::make_shared<candle_ros2::srv::SetMode::Response>();
    modeReq->device_ids = req->device_ids;
    modeReq->mode.assign(n, req->mode);
    cbSetMode(modeReq, modeRsp);

    auto zeroRsp = std::make_shared<candle_ros2::srv::Generic::Response>();
    if (initDevicesZero)
    {
        auto zeroReq        = std::make_shared<candle_ros2::srv::Generic::Request>();
        zeroReq->device_ids = req->device_ids;
        cbZero(zeroReq, zeroRsp);
    }
    else
    {
        zeroRsp->success.assign(n, true);
    }

    auto enableReq        = std::make_shared<candle_ros2::srv::Generic::Request>();
    auto enableRsp        = std::make_shared<candle_ros2::srv::Generic::Response>();
    enableReq->device_ids = req->device_ids;
    cbEnable(enableReq, enableRsp);

    for (size_t i = 0; i < n; i++)
    {
        const bool added   = i < addRsp->success.size() && addRsp->success[i];
        const bool modeOk  = i < modeRsp->success.size() && modeRsp->success[i];
        const bool zeroed  = i < zeroRsp->success.size() && zeroRsp->success[i];
        const bool enabled = i < enableRsp->success.size() && enableRsp->success[i];
        rsp->success[i]    = added && modeOk && zeroed && enabled;
    }
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

void MdNode::cbSetLimits(const std::shared_ptr<candle_ros2::srv::SetLimits::Request> req,
                         std::shared_ptr<candle_ros2::srv::SetLimits::Response>      rsp)
{
    if (req->device_ids.size() != req->velocity_limit.size() ||
        req->device_ids.size() != req->torque_limit.size())
    {
        rsp->success.assign(req->device_ids.size(), false);
        RCLCPP_WARN(this->get_logger(),
                    "SetLimits request incomplete. Sizes of arrays do not match!");
        return;
    }

    rsp->success.reserve(req->device_ids.size());
    for (size_t i = 0; i < req->device_ids.size(); i++)
    {
        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        mab::MDRegisters_S mdRegisters;
        mdRegisters.profileVelocity = req->velocity_limit[i];
        mdRegisters.maxTorque       = req->torque_limit[i];
        if (md->writeRegisters(mdRegisters.profileVelocity, mdRegisters.maxTorque) ==
            mab::MD::Error_t::OK)
        {
            rsp->success.push_back(true);
        }
        else
        {
            rsp->success.push_back(false);
        }
    }
    return;
}

bool MdNode::configureGripper(
    mab::MD& md, double kp, double kd, double velocityLimit, double torqueLimit)
{
    mab::MDRegisters_S impedanceRegs;
    impedanceRegs.motorImpPidKp = static_cast<float>(kp);
    impedanceRegs.motorImpPidKd = static_cast<float>(kd);
    impedanceRegs.maxTorque     = static_cast<float>(torqueLimit);
    if (md.writeRegisters(impedanceRegs.motorImpPidKp,
                          impedanceRegs.motorImpPidKd,
                          impedanceRegs.maxTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set impedance gains for drive with ID: %d", md.m_canId);
        return false;
    }

    mab::MDRegisters_S limitRegs;
    limitRegs.profileVelocity = static_cast<float>(velocityLimit);
    limitRegs.maxTorque       = static_cast<float>(torqueLimit);
    if (md.writeRegisters(limitRegs.profileVelocity, limitRegs.maxTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set limits for drive with ID: %d", md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::setGripperTarget(mab::MD& md, double targetPos)
{
    mab::MDRegisters_S motionRegs;
    motionRegs.targetPosition = targetPos;
    motionRegs.targetVelocity = 0.0;
    motionRegs.targetTorque   = 0.0;
    if (md.writeRegisters(motionRegs.targetPosition,
                          motionRegs.targetVelocity,
                          motionRegs.targetTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set target position for drive with ID: %d", md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::moveGripper(mab::MD& md, double targetPos)
{
    if (md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set IMPEDANCE mode for drive with ID: %d", md.m_canId);
        return false;
    }
    return configureGripper(md,
                            gripperImpedanceKp,
                            gripperImpedanceKd,
                            gripperVelocityLimitRadS,
                            gripperTorqueLimitNm) &&
           setGripperTarget(md, targetPos);
}

void MdNode::cbOpenGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                           std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", id);
            rsp->success.push_back(false);
            continue;
        }

        rsp->success.push_back(moveGripper(*md, gripperOpenPositionRad));
    }
}

void MdNode::cbCloseGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                            std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", id);
            rsp->success.push_back(false);
            continue;
        }

        rsp->success.push_back(moveGripper(*md, gripperClosedPositionRad));
    }
}

void MdNode::cbSetGripperTargets(
    const std::shared_ptr<candle_ros2::srv::SetGripperTargets::Request> req,
    std::shared_ptr<candle_ros2::srv::SetGripperTargets::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);
    if (n == 0 || req->target_position_rad.size() != n)
    {
        RCLCPP_WARN(this->get_logger(),
                    "SetGripperTargets request arrays must be non-empty and equally sized");
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        const double target = req->target_position_rad[i];
        if (!std::isfinite(target))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid gripper command values for drive with ID: %d",
                        req->device_ids[i]);
            continue;
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", req->device_ids[i]);
            continue;
        }
        rsp->success[i] = setGripperTarget(*md, target);
    }
}

void MdNode::cbConfigureGripper(
    const std::shared_ptr<candle_ros2::srv::ConfigureGripper::Request> req,
    std::shared_ptr<candle_ros2::srv::ConfigureGripper::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);
    if (n == 0 || req->kp.size() != n || req->kd.size() != n ||
        req->velocity_limit_rad_s.size() != n || req->torque_limit_nm.size() != n)
    {
        RCLCPP_WARN(this->get_logger(),
                    "ConfigureGripper request arrays must be non-empty and equally sized");
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        const double kp            = req->kp[i];
        const double kd            = req->kd[i];
        const double velocityLimit = req->velocity_limit_rad_s[i];
        const double torqueLimit   = req->torque_limit_nm[i];
        if (!std::isfinite(kp) || kp < 0.0 || !std::isfinite(kd) || kd < 0.0 ||
            !std::isfinite(velocityLimit) || velocityLimit <= 0.0 || !std::isfinite(torqueLimit) ||
            torqueLimit <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid gripper configuration for drive with ID: %d",
                        req->device_ids[i]);
            continue;
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", req->device_ids[i]);
            continue;
        }
        rsp->success[i] = configureGripper(*md, kp, kd, velocityLimit, torqueLimit);
    }
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

        if (md->setMotionMode(mode) == mab::MD::Error_t::OK)
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
