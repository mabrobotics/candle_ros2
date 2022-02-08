#include "md80_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cstdio>

Md80Node::Md80Node() : Node("md80_node")
{
	candle = new mab::Candle(mab::CANdleBaudrate_E::CAN_BAUD_1M, true);
	addMd80Service = this->create_service<ros2_mab_md80::srv::AddMd80s>(this->get_name() + std::string("/add_md80s"),
		std::bind(&Md80Node::service_addMd80, this, std::placeholders::_1, std::placeholders::_2));
	zeroMd80Service = this->create_service<ros2_mab_md80::srv::GenericMd80Msg>(this->get_name() + std::string("/zero_md80s"),
		std::bind(&Md80Node::service_zeroMd80, this, std::placeholders::_1, std::placeholders::_2));
	setModeMd80Service = this->create_service<ros2_mab_md80::srv::SetModeMd80s>(this->get_name() + std::string("/set_mode_md80s"),
		std::bind(&Md80Node::service_setModeMd80, this, std::placeholders::_1, std::placeholders::_2));
	enableMd80Service = this->create_service<ros2_mab_md80::srv::GenericMd80Msg>(this->get_name() + std::string("/enable_md80s"),
		std::bind(&Md80Node::service_enableMd80, this, std::placeholders::_1, std::placeholders::_2));
	disableMd80Service = this->create_service<ros2_mab_md80::srv::GenericMd80Msg>(this->get_name() + std::string("/disable_md80s"),
		std::bind(&Md80Node::service_disableMd80, this, std::placeholders::_1, std::placeholders::_2));

	jointStatePub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	pubTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Md80Node::publishJointStates, this));
	pubTimer->cancel();
	RCLCPP_INFO(this->get_logger(), "md80_node has started.");
}
Md80Node::~Md80Node()
{
	delete candle;
	RCLCPP_INFO(this->get_logger(), "md80_node finished.");
}
void Md80Node::service_addMd80(const std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Request> request,
					std::shared_ptr<ros2_mab_md80::srv::AddMd80s::Response> response)
{ 
	for(auto&id : request->drive_ids)
		response->drives_success.push_back(candle->addMd80(id));

	response->total_number_of_drives = candle->md80s.size();
}
void Md80Node::service_zeroMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response)
{
	for(auto&id : request->drive_ids)
		response->drives_success.push_back(candle->controlMd80SetEncoderZero(id));
}
void Md80Node::service_setModeMd80(const std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::SetModeMd80s::Response> response)
{
	if(request->drive_ids.size() != request->mode.size())
	{
		for(auto&id : request->drive_ids)
		{
			(void)id;
			response->drives_success.push_back(false);
		}
		RCLCPP_WARN(this->get_logger(), "Number of entires in 'drive_ids' and 'mode' does not match! Ignoring call.");
		return;
	}
	for(int i = 0; i < (int)request->drive_ids.size(); i++)
	{
		mab::Md80Mode_E mode;
		
		if(request->mode[i] == "IMPEDANCE")
			mode = mab::Md80Mode_E::IMPEDANCE;
		else if(request->mode[i] == "POSITION_PID")
			mode = mab::Md80Mode_E::POSITION_PID;
		else if(request->mode[i] == "VELOCITY_PID")
			mode = mab::Md80Mode_E::VELOCITY_PID;
		else if(request->mode[i] == "TORQUE")
			mode = mab::Md80Mode_E::TORQUE;
		else
		{
			RCLCPP_WARN(this->get_logger(), "MODE %s not recognized, setting IDLE for driveID = %d", request->mode[i].c_str(), request->drive_ids[i]);
			mode = mab::Md80Mode_E::IDLE;
		}
		response->drives_success.push_back(candle->controlMd80Mode(request->drive_ids[i], mode));
	}
}
void Md80Node::service_enableMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response)
{
	for(auto&id : request->drive_ids)
		response->drives_success.push_back(candle->controlMd80Enable(id,true));
	candle->begin();
	pubTimer->reset();
}
void Md80Node::service_disableMd80(const std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<ros2_mab_md80::srv::GenericMd80Msg::Response> response)
{
	candle->end();
	pubTimer->cancel();
	for(auto&id : request->drive_ids)
		response->drives_success.push_back(candle->controlMd80Enable(id,true));
}
void Md80Node::publishJointStates()
{
	sensor_msgs::msg::JointState jointStateMsg;
	jointStateMsg.header.stamp = rclcpp::Clock().now();
	for(auto&md : candle->md80s)
	{
		jointStateMsg.name.push_back(std::string("Joint " + std::to_string(md.getId())));
		jointStateMsg.position.push_back(md.getPosition());
		jointStateMsg.velocity.push_back(md.getVelocity());
		jointStateMsg.effort.push_back(md.getTorque());
	}
	this->jointStatePub->publish(jointStateMsg);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Md80Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}