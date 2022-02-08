#include "md80_node.hpp"

#include <cstdio>

Md80Node::Md80Node() : Node("md80_node")
{
	candle = new mab::Candle(mab::CANdleBaudrate_E::CAN_BAUD_1M, true);
	addMd80Service = this->create_service<ros2_mab_md80::srv::AddMd80s>("add_md80s",
		std::bind(&Md80Node::service_addMd80, this, std::placeholders::_1, std::placeholders::_2));
	this->imp.kp.push_back(3.0f);
	printf("DUPA %f\n", imp.kp[0]);
}
Md80Node::~Md80Node()
{
	printf("JAJA\n");
}
void Md80Node::service_addMd80(const std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Request> request,
					std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Response> response)
{ 
	for(auto&id : request->drive_ids)
	{
		std::cout << "Adding drive: " << id << std::endl;
		response->drives_added.push_back(true);
	}
	response->total_number_of_drives = response->drives_added.size();
}
void Md80Node::service_zeroMd80(const std::shared_ptr<ros2_mab_md80::srv::ZeroMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::ZeroMd80s::Response> response)
{
	for(auto&id : request->drive_ids)
	{
		//Candle.setZero
	}
}
void Md80Node::service_setModeMd80(const std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Response> response)
{
	if(request->drive_ids.size() != request->mode.size())
	{
		for(auto&id : request->drive_ids)
			response->drives_mode_set.push_back(false);
		return;
	}
	for(auto&id : request->drive_ids)
	{
		//Candle.setZero
	}
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Md80Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}