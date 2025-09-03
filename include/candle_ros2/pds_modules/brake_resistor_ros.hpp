#include "rclcpp/rclcpp.hpp"

#include "candle_ros2/msg/brake_resistor_data.hpp"

#include "pds.hpp"

class BrakeResistorRos : public BaseModuleRos
{
  public:
    bool setup(std::shared_ptr<rclcpp::Node> node,
               mab::Pds&                     pds,
               mab::socketIndex_E            socket,
               const int                     timerMs = 1000) override
    {
        parentNode = node;

        brakeResistor = pds.attachBrakeResistor(socket);
        if (brakeResistor == nullptr)
            return false;

        pubData = parentNode->create_publisher<candle_ros2::msg::BrakeResistorData>(
            "brake_resistor_" + std::to_string(static_cast<int>(socket)), 10);

        tmrPub = parentNode->create_wall_timer(std::chrono::milliseconds(timerMs),
                                               [this]() { this->publishStatus(); });

        return true;
    }

    void publishStatus()
    {
        auto msg = candle_ros2::msg::BrakeResistorData();

        msg.header.stamp = parentNode->get_clock()->now();

        brakeResistor->getEnabled(msg.enabled);
        brakeResistor->getTemperature(msg.temperature);
        brakeResistor->getTemperatureLimit(msg.temperature_limit);

        pubData->publish(msg);
    }

  private:
    std::shared_ptr<mab::BrakeResistor> brakeResistor;

    rclcpp::Publisher<candle_ros2::msg::BrakeResistorData>::SharedPtr pubData;
};
