#include "candle_ros2/pds_node.hpp"

PdsNode::PdsNode() : Node("candle_pds_node")
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
                    "<data_rate> parameter not recognised! Value: '%s'",
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
            this->get_logger(), "<bus> parameter not recognised! Value: %s", paramBus.c_str());
        return;
    }

    candle = std::unique_ptr<mab::Candle>(mab::attachCandle(dataRate, bus));

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
        auto instance = PdsInstance{};
        instance.pds  = std::make_unique<mab::Pds>(id, candle.get());
        instance.pds->init();
        /*
        ******************************************************
            TODO: After CANdle-SDK update, add safety checks
        ******************************************************
        */

        mab::Pds::modulesSet_S pdsModules = instance.pds->getModules();

        using ModulePtr = mab::moduleType_E mab::Pds::modulesSet_S::*;

        ModulePtr sockets[] = {&mab::Pds::modulesSet_S::moduleTypeSocket1,
                               &mab::Pds::modulesSet_S::moduleTypeSocket2,
                               &mab::Pds::modulesSet_S::moduleTypeSocket3,
                               &mab::Pds::modulesSet_S::moduleTypeSocket4,
                               &mab::Pds::modulesSet_S::moduleTypeSocket5,
                               &mab::Pds::modulesSet_S::moduleTypeSocket6};

        auto ctrl = std::make_unique<ControlModuleRos>();
        ctrl->setup(shared_from_this(),
                    *instance.pds,
                    mab::socketIndex_E::UNASSIGNED,
                    id,
                    NODE_PREFIX,
                    PUB_TIMER_MS);

        // instance.ctrlModule = std::move(ctrl);
        instance.modules.push_back(std::move(ctrl));

        for (int i = 0; i < 6; i++)
        {
            mab::moduleType_E type = pdsModules.*(sockets[i]);
            auto              mod  = createModule(type);
            // TODO: check if module was created
            if (mod)
            {
                // TODO: check return value (bool)
                mod->setup(shared_from_this(),
                           *instance.pds,
                           static_cast<mab::socketIndex_E>(i + 1),
                           id,
                           NODE_PREFIX,
                           PUB_TIMER_MS);
                instance.modules.push_back(std::move(mod));
            }
        }

        pds_list.emplace_back(std::move(instance));

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
    rsp->total_devices = static_cast<u16>(pds_list.size());
}

void PdsNode::cbReboot(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                       std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto it = std::find_if(pds_list.begin(),
                               pds_list.end(),
                               [id](const PdsInstance& instance)
                               {
                                   if (!instance.pds)
                                       return false;
                                   return instance.pds->getCanId() == id;
                               });

        if (it != pds_list.end())
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
        auto it = std::find_if(pds_list.begin(),
                               pds_list.end(),
                               [id](const PdsInstance& instance)
                               {
                                   if (!instance.pds)
                                       return false;
                                   return instance.pds->getCanId() == id;
                               });

        if (it != pds_list.end())
        {
            if (it->pds->shutdown() == mab::PdsModule::error_E::OK)
            {
                rsp->success.push_back(true);
                pds_list.erase(it);
            }
            else
                rsp->success.push_back(false);
        }
        else
            rsp->success.push_back(false);
    }

    return;
}

std::unique_ptr<BaseModuleRos> PdsNode::createModule(mab::moduleType_E type)
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

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PdsNode>());
    rclcpp::shutdown();
    return 0;
}
