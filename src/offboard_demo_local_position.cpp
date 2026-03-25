#include <px4_msgs/msg/offboard_control_mode.hpp>   // 消息类型：离板控制模式（告知PX4进入离板模式）
#include <px4_msgs/msg/trajectory_setpoint.hpp>     // 消息类型：轨迹设定点（下发位置/速度/高度等飞行指令）
#include <px4_msgs/msg/vehicle_command.hpp>         // 消息类型：载具指令（下发起飞/降落/解锁/锁定等指令）
#include <px4_msgs/msg/vehicle_control_mode.hpp>    // 消息类型：载具控制模式（获取飞控当前的控制模式状态）
#include <px4_msgs/msg/vehicle_local_position.hpp>  // 消息类型：载具本地位置（位置、速度等状态信息）
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
        // 发布器实例化：离板控制模式、目标航点设定、载具指令
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        // 订阅器实例化：载具本地位置
        vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

		offboard_setpoint_counter_ = 0;
        second_counter_ = 0;

		auto timer_callback = [this]() -> void 
		{
            second_counter_++;
            if (second_counter_ >= 10)
            {
                RCLCPP_INFO(this->get_logger(), "The current height is %.2f m", -(current_z_.load()));
                second_counter_ = 0;
            }
            
			if (offboard_setpoint_counter_ == 10) 
			{
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			if (offboard_setpoint_counter_ < 11) 
			{
				offboard_setpoint_counter_++;
			}
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);        // 成员函数：每 100ms 撞墙触发回调函数
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

    // 声明发布器：离板控制模式、目标航点设定、载具指令
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    // 声明订阅器：载具本地位置
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

	std::atomic<uint64_t> timestamp_;      // 原子类型（保证多线程安全）：公共同步时间戳
    uint8_t offboard_setpoint_counter_;    // Offboard 切换计数器
    uint8_t second_counter_;               // 秒计数器
    std::atomic<float> current_x_{0.0f};
    std::atomic<float> current_y_{0.0f};
    std::atomic<float> current_z_{0.0f};

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(); 
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
};

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;                                                // 位置控制
	msg.velocity = false;                                               // 速度控制
	msg.acceleration = false;                                           // 加速度控制
	msg.attitude = false;                                               // 姿态控制
	msg.body_rate = false;                                              // 角速度控制
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;      // 时间戳转换：ROS2 纳秒 --> PX4 微秒

	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0, 0.0, -5.0};
	msg.yaw = 3.14 / 2;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;

    // NED 坐标系，用于前期调试
    // RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f, %.2f]", current_x_.load(), current_y_.load(), current_z_.load());   
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}