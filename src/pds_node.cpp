#include "candle_ros2/pds_node.hpp"

PdsNode::PdsNode() : Node("candle_pds_node")
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
        RCLCPP_INFO(
            this->get_logger(), "<baud> parameter not recognised! Value: '%s'", paramBaud.c_str());
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

    candle = std::unique_ptr<mab::Candle>(mab::attachCandle(baud, bus));

    srvAddPds = this->create_service<candle_ros2::srv::AddDevices>(
        topicPrefix + "add_pds",
        std::bind(&PdsNode::cbAddPds, this, std::placeholders::_1, std::placeholders::_2));

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
        mab::Pds pds(id, candle.get());

        pds.init();
        /*
        ******************************************************
            TODO: After CANdle-SDK update, add safety checks
        ******************************************************
        */

        mab::Pds::modulesSet_S pdsModules = pds.getModules();

        PdsInstance instance(std::move(pds));

        using ModulePtr = mab::moduleType_E mab::Pds::modulesSet_S::*;

        ModulePtr sockets[] = {&mab::Pds::modulesSet_S::moduleTypeSocket1,
                               &mab::Pds::modulesSet_S::moduleTypeSocket2,
                               &mab::Pds::modulesSet_S::moduleTypeSocket3,
                               &mab::Pds::modulesSet_S::moduleTypeSocket4,
                               &mab::Pds::modulesSet_S::moduleTypeSocket5,
                               &mab::Pds::modulesSet_S::moduleTypeSocket6};

        for (int i = 0; i < 6; i++)
        {
            mab::moduleType_E type = pdsModules.*(sockets[i]);
            auto              mod  = createModule(type);
            // TODO: check if module was created
            if (mod)
            {
                // TODO: check return value (bool)
                mod->setup(shared_from_this(),
                           instance.pds,
                           static_cast<mab::socketIndex_E>(i + 1),
                           PUB_TIMER_MS);
                instance.modules.push_back(std::move(mod));
            }
        }

        pds_list.emplace_back(std::move(instance));
        rsp->success.push_back(true);
    }

    rsp->total_devices = static_cast<u16>(pds_list.size());
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
