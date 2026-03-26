#include <px4_msgs/msg/offboard_control_mode.hpp>		// 消息类型：离板控制模式（告知 PX4 进入离板模式）
#include <px4_msgs/msg/trajectory_setpoint.hpp>			// 消息类型：轨迹设定点（下发位置/速度/高度等飞行指令）
#include <px4_msgs/msg/vehicle_command.hpp>				// 消息类型：载具指令（下发起飞/降落/解锁/锁定等指令）
#include <px4_msgs/msg/vehicle_control_mode.hpp>		// 消息类型：载具控制模式（飞控当前的控制模式状态）
#include <px4_msgs/msg/vehicle_local_position.hpp>		// 消息类型：载具本地位置（位置、速度等状态信息）
#include <px4_msgs/msg/vehicle_status.hpp>  			// 新增：载具状态消息类型(飞行模式、是否降落等状态信息)
#include <rclcpp/rclcpp.hpp>							// ROS2核心库
#include <stdint.h>										// 标准整数类型定义
#include <chrono>										// 时间库
#include <iostream>										// 输入输出库

using namespace std::chrono;							// 时间命名空间
using namespace std::chrono_literals;					// 时间字面量
using namespace px4_msgs::msg;							// PX4消息命名空间

class OffboardControl : public rclcpp::Node				// 定义离板控制类，继承自ROS2节点
{
public:													// 公共成员函数
	OffboardControl() : Node("offboard_control")
	{
		// 发布器实例化：离板控制模式、目标航点设定、载具指令
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		// 变量 = 指针<消息类型>(订阅话题，QoS，回调函数)
        
        // 订阅器实例化：本地位置信息、载具状态信息
        vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
        vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
		// 变量 = 指针<消息类型>(订阅话题，QoS，回调函数)

		// 初始化：计数器
		offboard_setpoint_counter_ = 0;					// 离板切换计数器
        second_counter_ = 0;							// 秒计数器
        is_land_triggered_ = false;						// 触发降落标志位
        is_disarmed_ = false;							// 上锁标志位
        target_height_ = -5.0f;							// 目标高度

		// 定时回调函数（100ms）
		auto timer_callback = [this]() -> void 
		{
            second_counter_++;
			// 每秒打印一次当前高度
            if (second_counter_ >= 10)
            {
                RCLCPP_INFO(this->get_logger(), "The current height is %.2f m", fabs(current_z_.load()));
                second_counter_ = 0;
            }

			// offboard 切换计数器
			if (offboard_setpoint_counter_ < 11) 
			{
				offboard_setpoint_counter_++;
			}

            // offboard 切换
			if (offboard_setpoint_counter_ == 10) 
			{
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			// 发布 offboard 模式
			publish_offboard_control_mode();
            
			// 发布目标航点
            if (!is_land_triggered_) 
			{
                publish_trajectory_setpoint();
            }

            // 达到目标高度后触发降落
            if (!is_land_triggered_ && fabs(current_z_.load()) >= fabs(target_height_) - 0.1) 
			{
                RCLCPP_INFO(this->get_logger(), "Reach target height, trigger land...");
                this->land();  // 触发降落
                is_land_triggered_ = true;
            }

            // 降落完成后上锁并退出程序
            if (is_land_triggered_ && !is_disarmed_ && fabs(current_z_.load()) <= 0.2) 
			{
                RCLCPP_INFO(this->get_logger(), "Landing completed, disarm...");
                this->disarm();
                is_disarmed_ = true;
                rclcpp::shutdown();
                exit(0);
            }
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();											// 解锁
	void disarm();										// 上锁
    void land();										// 降落

private:
	rclcpp::TimerBase::SharedPtr timer_;

	// 发布器：离板控制模式、目标航点设定、载具指令
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    // 订阅器：载具本地位置、载具状态
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;  // + 新增

	std::atomic<uint64_t> timestamp_;					// 原子类型：公共同步时间戳
    uint8_t offboard_setpoint_counter_;					// 离板切换计数器
    uint8_t second_counter_;							// 秒计数器
    std::atomic<float> current_x_{0.0f};
    std::atomic<float> current_y_{0.0f};
    std::atomic<float> current_z_{0.0f};				// 原子类型：当前高度
    bool is_land_triggered_;    						// 是否已触发降落
    bool is_disarmed_;          						// 是否已上锁
    float target_height_;       						// 目标高度
    VehicleStatus::SharedPtr vehicle_status_;  			// 存储载具状态

	// 声明成员函数：发布离板控制模式、发布目标航点、发布载具指令、载具本地位置回调、载具状态回调
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(); 
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
};

// 解锁
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

// 上锁
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

// 降落
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}

// 发布 Offboard 控制模式
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// 发布 5m 高度设定点
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0, 0.0, -5.0};  // ✏️ 确认目标高度（5m对应z=-5.0）
	msg.yaw = 3.14 / 2;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

// 发布载具指令
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

// 本地位置回调
void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
}

// + 载具状态回调
void OffboardControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    vehicle_status_ = msg;
}

// 主函数
int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}