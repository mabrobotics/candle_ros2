#include "rclcpp/rclcpp.hpp"

#include "candle_ros2/msg/power_stage_data.hpp"

#include "pds.hpp"

class PowerStageRos : public BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     pdsId,
               const std::string&            nodePrefix = "pds/",
               const int                     timerMs    = 1000) override
    {
        parentNode = node;

        powerStage = pds.attachPowerStage(socket);
        if (powerStage == nullptr)
            return false;

        pubData = parentNode->create_publisher<candle_ros2::msg::PowerStageData>(
            nodePrefix + std::to_string(pdsId) + "/power_stage_" +
                std::to_string(static_cast<int>(socket)),
            10);

        tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                               [this]() { this->publishStatus(); });

        return true;
    }

    void publishStatus()
    {
        auto msg = candle_ros2::msg::PowerStageData();

        msg.header.stamp = parentNode->get_clock()->now();

        powerStage->getEnabled(msg.enabled);

        mab::socketIndex_E sck;
        powerStage->getBindBrakeResistor(sck);
        msg.brake_resistor_socket = static_cast<uint8_t>(sck);

        powerStage->getBrakeResistorTriggerVoltage(msg.trigger_voltage);
        powerStage->getOutputVoltage(msg.output_voltage);
        powerStage->getAutostart(msg.autostart);
        powerStage->getLoadCurrent(msg.load_current);
        powerStage->getPower(msg.power);
        powerStage->getTotalDeliveredEnergy(msg.energy);
        powerStage->getOcdLevel(msg.ocd_level);
        powerStage->getOcdDelay(msg.ocd_delay);
        powerStage->getTemperature(msg.temperature);
        powerStage->getTemperatureLimit(msg.temperature_limit);

        pubData->publish(msg);
    }

  private:
    std::shared_ptr<mab::PowerStage> powerStage;

    rclcpp::Publisher<candle_ros2::msg::PowerStageData>::SharedPtr pubData;
};