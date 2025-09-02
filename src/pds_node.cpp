#include "pds_node.hpp"

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

        RCLCPP_INFO(
            this->get_logger(), "PDS with ID %d has the following set of connected modules:", id);
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 1 :: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket1));
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 2: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket2));
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 3: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket3));
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 4: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket4));
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 5: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket5));
        RCLCPP_INFO(this->get_logger(),
                    "\tSocket 6: %s",
                    mab::Pds::moduleTypeToString(pdsModules.moduleTypeSocket6));

        switch (pdsModules.moduleTypeSocket1)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_1);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_1);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_1);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        switch (pdsModules.moduleTypeSocket2)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_2);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_2);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_2);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        switch (pdsModules.moduleTypeSocket3)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_3);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_3);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_3);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        switch (pdsModules.moduleTypeSocket4)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_4);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_4);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_4);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        switch (pdsModules.moduleTypeSocket5)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_5);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_5);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_5);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        switch (pdsModules.moduleTypeSocket6)
        {
            case mab::moduleType_E::BRAKE_RESISTOR:
                pds.attachBrakeResistor(mab::socketIndex_E::SOCKET_6);
                break;
            case mab::moduleType_E::ISOLATED_CONVERTER:
                pds.attachIsolatedConverter(mab::socketIndex_E::SOCKET_6);
                break;
            case mab::moduleType_E::POWER_STAGE:
                pds.attachPowerStage(mab::socketIndex_E::SOCKET_6);
                break;
            case mab::moduleType_E::CONTROL_BOARD:
                break;
            case mab::moduleType_E::UNDEFINED:
                break;
            default:
                break;
        }

        pds_list.push_back(std::move(pds));
        rsp->success.push_back(true);
    }
    rsp->total_devices = static_cast<u16>(pds_list.size());
    return;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PdsNode>());
    rclcpp::shutdown();
    return 0;
}
