#include "candle_ros2/pds_node.hpp"

PdsNode::PdsNode(const rclcpp::NodeOptions&   options,
                 std::shared_ptr<mab::Candle> candle,
                 const candleParams_S&        params)
    : Node("candle_pds_node", options), m_candle(candle), m_defaultQoS(10)
{
    m_defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        m_defaultQoS.best_effort();

    srvAddPds = this->create_service<candle_ros2::srv::AddDevices>(
        std::string(NODE_PREFIX) + "add_pds",
        std::bind(&PdsNode::cbAddPds, this, std::placeholders::_1, std::placeholders::_2));
    srvReboot = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "reboot_pds",
        std::bind(&PdsNode::cbReboot, this, std::placeholders::_1, std::placeholders::_2));
    srvShutdown = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "shutdown_pds",
        std::bind(&PdsNode::cbShutdown, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 PDS node started.");
}

PdsNode::~PdsNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 PDS node finished.");
}

void PdsNode::cbAddPds(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                       std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        pdsInstance_S instance = pdsInstance_S{};

        instance.pds = std::make_unique<mab::Pds>(id, m_candle.get());

        if (instance.pds->init() != mab::PdsModule::error_E::OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to init PDS with id %d", id);
            rsp->success.push_back(false);
            continue;
        }

        mab::Pds::modulesSet_S pdsModules = instance.pds->getModules();

        using ModulePtr = mab::moduleType_E mab::Pds::modulesSet_S::*;

        ModulePtr sockets[] = {&mab::Pds::modulesSet_S::moduleTypeSocket1,
                               &mab::Pds::modulesSet_S::moduleTypeSocket2,
                               &mab::Pds::modulesSet_S::moduleTypeSocket3,
                               &mab::Pds::modulesSet_S::moduleTypeSocket4,
                               &mab::Pds::modulesSet_S::moduleTypeSocket5,
                               &mab::Pds::modulesSet_S::moduleTypeSocket6};

        auto ctrl = std::make_unique<ControlModuleRos>();
        if (ctrl->setup(shared_from_this(),
                        *instance.pds,
                        mab::socketIndex_E::UNASSIGNED,
                        id,
                        m_defaultQoS,
                        NODE_PREFIX,
                        PUB_TIMER_MS))
            instance.modules.push_back(std::move(ctrl));
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup control module for PDS id %d", id);
            rsp->success.push_back(false);
            continue;
        }

        for (int i = 0; i < 6; i++)
        {
            mab::moduleType_E type = pdsModules.*(sockets[i]);
            auto              mod  = createModule(type);

            if (mod)
            {
                if (mod->setup(shared_from_this(),
                               *instance.pds,
                               static_cast<mab::socketIndex_E>(i + 1),
                               id,
                               m_defaultQoS,
                               NODE_PREFIX,
                               PUB_TIMER_MS))
                    instance.modules.push_back(std::move(mod));
            }
        }

        m_pdsList.emplace_back(std::move(instance));

        RCLCPP_INFO(
            this->get_logger(), "PDS with id %d have the following set of connected modules:", id);
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 1: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket1));
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 2: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket2));
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 3: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket3));
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 4: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket4));
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 5: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket5));
        RCLCPP_INFO(this->get_logger(),
                    "- Socket 6: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket6));

        rsp->success.push_back(true);
    }
    rsp->total_devices = static_cast<u16>(m_pdsList.size());
}

void PdsNode::cbReboot(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                       std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto it = std::find_if(m_pdsList.begin(),
                               m_pdsList.end(),
                               [id](const pdsInstance_S& instance)
                               {
                                   if (!instance.pds)
                                       return false;
                                   return instance.pds->getCanId() == id;
                               });

        if (it != m_pdsList.end())
        {
            if (it->pds->reboot() == mab::PdsModule::error_E::OK)
                rsp->success.push_back(true);
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }

    return;
}

void PdsNode::cbShutdown(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                         std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto it = std::find_if(m_pdsList.begin(),
                               m_pdsList.end(),
                               [id](const pdsInstance_S& instance)
                               {
                                   if (!instance.pds)
                                       return false;
                                   return instance.pds->getCanId() == id;
                               });

        if (it != m_pdsList.end())
        {
            if (it->pds->shutdown() == mab::PdsModule::error_E::OK)
            {
                rsp->success.push_back(true);
                m_pdsList.erase(it);
            }
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }

    return;
}

std::unique_ptr<I_BaseModuleRos> PdsNode::createModule(mab::moduleType_E type)
{
    switch (type)
    {
        case mab::moduleType_E::BRAKE_RESISTOR:
            return std::make_unique<BrakeResistorRos>();
        case mab::moduleType_E::POWER_STAGE:
            return std::make_unique<PowerStageRos>();
        case mab::moduleType_E::ISOLATED_CONVERTER:
            return std::make_unique<IsolatedConverterRos>();
        case mab::moduleType_E::CONTROL_BOARD:
        case mab::moduleType_E::UNDEFINED:
            break;
        default:
            break;
    }
    return nullptr;
}
