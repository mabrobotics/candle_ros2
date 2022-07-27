#include "md80_sim_node.hpp"

const std::string version = "v1.1";

Md80Node::Md80Node() : Node("candle_ros2_node")
{
	std::string homedir = getenv("HOME");
	std::string file_string = homedir + "/log/latest/candle_log_sim.csv";
	RCLCPP_INFO(this->get_logger(), file_string);
	logFile.open(file_string, std::fstream::out);
    logFile << "frame_id, sec, nanosec\n";
	addMd80Service = this->create_service<candle_ros2::srv::AddMd80s>(this->get_name() + std::string("/add_md80s"),
		std::bind(&Md80Node::service_addMd80, this, std::placeholders::_1, std::placeholders::_2));
	zeroMd80Service = this->create_service<candle_ros2::srv::GenericMd80Msg>(this->get_name() + std::string("/zero_md80s"),
		std::bind(&Md80Node::service_zeroMd80, this, std::placeholders::_1, std::placeholders::_2));
	setModeMd80Service = this->create_service<candle_ros2::srv::SetModeMd80s>(this->get_name() + std::string("/set_mode_md80s"),
		std::bind(&Md80Node::service_setModeMd80, this, std::placeholders::_1, std::placeholders::_2));
	enableMd80Service = this->create_service<candle_ros2::srv::GenericMd80Msg>(this->get_name() + std::string("/enable_md80s"),
		std::bind(&Md80Node::service_enableMd80, this, std::placeholders::_1, std::placeholders::_2));
	disableMd80Service = this->create_service<candle_ros2::srv::GenericMd80Msg>(this->get_name() + std::string("/disable_md80s"),
		std::bind(&Md80Node::service_disableMd80, this, std::placeholders::_1, std::placeholders::_2));

	motionCommandSub = this->create_subscription<candle_ros2::msg::MotionCommand>("md80/motion_command", 10,
		std::bind(&Md80Node::motionCommandCallback, this, std::placeholders::_1));
	impedanceCommandSub = this->create_subscription<candle_ros2::msg::ImpedanceCommand>("md80/impedance_command", 10,
		std::bind(&Md80Node::impedanceCommandCallback, this, std::placeholders::_1));
	velocityCommandSub = this->create_subscription<candle_ros2::msg::VelocityPidCommand>("md80/velocity_pid_command", 10,
		std::bind(&Md80Node::velocityCommandCallback, this, std::placeholders::_1));
	positionCommandSub = this->create_subscription<candle_ros2::msg::PositionPidCommand>("md80/position_pid_command", 10,
		std::bind(&Md80Node::positionCommandCallback, this, std::placeholders::_1));

	jointStatePub = this->create_publisher<sensor_msgs::msg::JointState>("md80/joint_states", 10);
	pubTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Md80Node::publishJointStates, this));
	pubTimer->cancel();
	RCLCPP_INFO(this->get_logger(), "candle_ros2_node %s has started.", version.c_str());
}
Md80Node::~Md80Node()
{
	RCLCPP_INFO(this->get_logger(), "candle_ros2_node finished.");
}
void Md80Node::service_addMd80(const std::shared_ptr<candle_ros2::srv::AddMd80s::Request> request,
					std::shared_ptr<candle_ros2::srv::AddMd80s::Response> response)
{ 
	for(auto id : request->drive_ids){
		response->drives_success.push_back(true);
		motors.push_back(id);
		RCLCPP_INFO(this->get_logger(), "added motors: '%d'", id);
	}

	response->total_number_of_drives = response->drives_success.size();

}
void Md80Node::service_zeroMd80(const std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Response> response)
{
	for(auto&id : request->drive_ids){
		response->drives_success.push_back(true);
		RCLCPP_INFO(this->get_logger(), "zero motors: '%d'", id);
	}

	
}

void Md80Node::service_setModeMd80(const std::shared_ptr<candle_ros2::srv::SetModeMd80s::Request> request,
        std::shared_ptr<candle_ros2::srv::SetModeMd80s::Response> response)
{
	
	if(request->drive_ids.size() != request->mode.size())
	{
		
		for(auto&id : request->drive_ids)
		{
			(void)id;
			response->drives_success.push_back(false);
		}
		RCLCPP_WARN(this->get_logger(), "SetMode request incomplete. Sizes of arrays do not match!");
		return;
	}
	for(int i = 0; i < (int)request->drive_ids.size(); i++)
	{
		
		mab::Md80Mode_E mode;
		
		if(request->mode[i] == "IMPEDANCE"){
			
			mode = mab::Md80Mode_E::IMPEDANCE;
		}
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
		response->drives_success.push_back(true);
		RCLCPP_INFO(this->get_logger(), "Motor %d changed mode to %s", request->drive_ids[i], request->mode[i]);
		
	}
}

void Md80Node::service_enableMd80(const std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Response> response)
{
	for(auto&id : request->drive_ids){
		RCLCPP_INFO(this->get_logger(), "enable motors: '%d'", id);
	}
	pubTimer->reset();
	
}

void Md80Node::service_disableMd80(const std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Request> request,
        std::shared_ptr<candle_ros2::srv::GenericMd80Msg::Response> response)
{
	pubTimer->cancel();
	for(auto&id : request->drive_ids)
		RCLCPP_INFO(this->get_logger(), "disable motors: '%d'", id);

	
}

void Md80Node::publishJointStates()
{

	sensor_msgs::msg::JointState jointStateMsg;
	jointStateMsg.header.stamp = rclcpp::Clock().now();

	for(auto i : motors)
	{

		jointStateMsg.name.push_back(std::string("Joint " + std::to_string(i)));
		jointStateMsg.position.push_back(0.01);
		jointStateMsg.velocity.push_back(0.02);
		jointStateMsg.effort.push_back(0.03);
	}
	this->jointStatePub->publish(jointStateMsg);
}

void Md80Node::motionCommandCallback(const std::shared_ptr<candle_ros2::msg::MotionCommand> msg)
{
	if(msg->drive_ids.size() != msg->target_position.size() || msg->drive_ids.size() != msg->target_velocity.size() ||
		msg->drive_ids.size() != msg->target_torque.size() ||
		msg->drive_ids.size() != msg->kp.size() || msg->drive_ids.size() != msg->kd.size())
	{
		RCLCPP_WARN(this->get_logger(), "Motion Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}

	builtin_interfaces::msg::Time t = rclcpp::Clock().now();
    logFile << msg->header.frame_id << "," << t.sec << "," << t.nanosec << "\n";

	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			RCLCPP_INFO(this->get_logger(), "id: %d, pos: %f, vel: %f, torque: %f, kp %f, kd %f",
			msg->drive_ids[i], 
			msg->target_position[i], 
			msg->target_velocity[i], 
			msg->target_torque[i],
			msg->kp[i],
			msg->kd[i]
			);
		}
		catch(const char* eMsg)
		{
			RCLCPP_WARN(this->get_logger(), eMsg);
		}
		

	}
}

void Md80Node::impedanceCommandCallback(const std::shared_ptr<candle_ros2::msg::ImpedanceCommand> msg)
{
	if(msg->drive_ids.size() != msg->kp.size() || msg->drive_ids.size() != msg->kd.size())
	{
		RCLCPP_WARN(this->get_logger(), "Impedance Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			RCLCPP_INFO(this->get_logger(), "impedance id: %d, kp: %f, kd: %f, max_torque: %f", 
			msg->drive_ids[i], msg->kp[i], msg->kd[i], msg->max_output[i]);
		}
		catch(const char* eMsg)
		{
			RCLCPP_WARN(this->get_logger(), eMsg);
		}	
	}
}

void Md80Node::velocityCommandCallback(const std::shared_ptr<candle_ros2::msg::VelocityPidCommand> msg)
{
	if(msg->drive_ids.size() != msg->velocity_pid.size())
	{
		RCLCPP_WARN(this->get_logger(), "Velocity Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			RCLCPP_INFO(this->get_logger(), "velocity id: %d, kp: %f, ki: %f, kd: %f, i_windup: %f, max_output: %f", 
			msg->drive_ids[i], 
			msg->velocity_pid[i].kp, 
			msg->velocity_pid[i].ki, 
			msg->velocity_pid[i].kd, 
			msg->velocity_pid[i].i_windup,
			msg->velocity_pid[i].max_output);
		}
		catch(const char* eMsg)
		{
			RCLCPP_WARN(this->get_logger(), eMsg);
		}	
	}
}
void Md80Node::positionCommandCallback(const std::shared_ptr<candle_ros2::msg::PositionPidCommand> msg)
{
	if(msg->drive_ids.size() != msg->position_pid.size())
	{
		RCLCPP_WARN(this->get_logger(), "Position Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			RCLCPP_INFO(this->get_logger(), "velocity id: %d, kp: %f, ki: %f, kd: %f, i_windup: %f, max_output: %f", 
			msg->drive_ids[i],
			msg->position_pid[i].kp, 
			msg->position_pid[i].ki, 
			msg->position_pid[i].kd, 
			msg->position_pid[i].i_windup,
			msg->position_pid[i].max_output
			);
			
		}
		catch(const char* eMsg)
		{
			RCLCPP_WARN(this->get_logger(), eMsg);
		}	

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