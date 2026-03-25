/**
 * @brief 离线控制任务示例
 * @file offboard_demo.cpp
 * @addtogroup examples
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief 飞行状态枚举
 */
enum FlightState {
    INIT,               // 初始化
    ARM_AND_TAKEOFF,    // 解锁起飞
    HOVER_1,            // 悬停阶段1
    ROTATE_360,         // 360度旋转
    HOVER_2,            // 悬停阶段2
    LAND,               // 降落
    DISARM,             // 上锁
    FINISHED            // 任务完成
};

/**
 * @brief 离线控制演示类
 */
class OffboardDemo : public rclcpp::Node
{
public:
    OffboardDemo() : Node("offboard_demo")
    {
        // 初始化发布器
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        // 状态变量初始化
        offboard_setpoint_counter_ = 0;
        flight_state_ = FlightState::INIT;
        state_start_time_ = this->now();
        land_command_sent_ = false;  // 新增：降落指令发送标志
        
        // 目标位置和偏航角
        target_position_[0] = 0.0f;  // 北向
        target_position_[1] = 0.0f;  // 东向
        target_position_[2] = -5.0f; // 向下（-5米）
        target_yaw_ = 0.0f;
        
        // 100ms定时器
        auto timer_callback = [this]() -> void 
        {
            auto current_time = this->now();
            auto state_duration = (current_time - state_start_time_).seconds();
            
            switch (flight_state_) 
            {
                case FlightState::INIT:
                {
                    if (offboard_setpoint_counter_ < 10) 
                    {
                        publish_offboard_control_mode();
                        publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], 0.0f);
                        offboard_setpoint_counter_++;
                    } 
                    else if (offboard_setpoint_counter_ == 10) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode");
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                        
                        RCLCPP_INFO(this->get_logger(), "Arming");
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                        
                        offboard_setpoint_counter_++;
                        flight_state_ = FlightState::ARM_AND_TAKEOFF;
                        state_start_time_ = this->now();
                    }
                    break;
                }
                
                case FlightState::ARM_AND_TAKEOFF:
                {
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], target_yaw_);
                    
                    if (state_duration > 10.0) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Raising to 5m, hovering for 10s");
                        flight_state_ = FlightState::HOVER_1;
                        state_start_time_ = this->now();
                    }
                    break;
                }
                
                case FlightState::HOVER_1:
                {
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], target_yaw_);
                    
                    if (state_duration > 10.0) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Starting 360-degree rotation");
                        flight_state_ = FlightState::ROTATE_360;
                        state_start_time_ = this->now();
                        rotation_start_yaw_ = target_yaw_;
                    }
                    break;
                }
                
                case FlightState::ROTATE_360:
                {
                    float rotation_time = 10.0f;
                    float elapsed_ratio = std::min(state_duration / rotation_time, 1.0);
                    float rotation_angle = rotation_start_yaw_ + 2.0f * M_PI * elapsed_ratio;
                    
                    while (rotation_angle > M_PI) rotation_angle -= 2.0f * M_PI;
                    while (rotation_angle < -M_PI) rotation_angle += 2.0f * M_PI;
                    
                    target_yaw_ = rotation_angle;
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], target_yaw_);
                    
                    if (state_duration >= rotation_time) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Rotation complete, hovering for 10s");
                        flight_state_ = FlightState::HOVER_2;
                        state_start_time_ = this->now();
                        target_yaw_ = rotation_start_yaw_;
                    }
                    break;
                }
                
                case FlightState::HOVER_2:
                {
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], target_yaw_);
                    
                    if (state_duration > 10.0) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Starting landing");
                        flight_state_ = FlightState::LAND;
                        state_start_time_ = this->now();
                    }
                    break;
                }
                
                case FlightState::LAND:
                {
                    if (!land_command_sent_) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Sending LAND command");
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                        land_command_sent_ = true;
                    }
                    
                    if (state_duration > 10.0) 
                    {
                        flight_state_ = FlightState::DISARM;
                        state_start_time_ = this->now();
                    }
                    break;
                }
                
                case FlightState::DISARM:
                {
                    if (state_duration > 5.0) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Disarming");
                        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                        flight_state_ = FlightState::FINISHED;
                        RCLCPP_INFO(this->get_logger(), "Mission complete!!!");
                    }
                    break;
                }
                
                case FlightState::FINISHED:
                {
                    RCLCPP_INFO_ONCE(this->get_logger(), "Press Ctrl+C to exit.");
                    break;
                }
            }
        };
        
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    uint64_t offboard_setpoint_counter_;  // 设定点计数器
    FlightState flight_state_;            // 当前状态
    rclcpp::Time state_start_time_;       // 状态开始时间
    float target_position_[3];            // 目标位置
    float target_yaw_;                    // 目标偏航角
    float rotation_start_yaw_;            // 旋转起始角度
    bool land_command_sent_;              // 降落指令发送标志
    
    /**
     * @brief 发布离线控制模式
     */
    void publish_offboard_control_mode()
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
    
    /**
     * @brief 发布轨迹设定点
     */
    void publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }
    
    /**
     * @brief 发布载具指令
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param7 = param7;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard demo node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardDemo>());
    rclcpp::shutdown();
    return 0;
}