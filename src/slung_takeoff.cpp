#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
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
        // 发布器实例化
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
                RCLCPP_INFO(this->get_logger(), "Current Z: %.2f m | Target Z: %.2f m", 
                            -(current_z_.load()), -target_z_);
                second_counter_ = 0;
            }
            
            if (offboard_setpoint_counter_ == 10) 
            {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
                
                // 【修改 1】在起飞瞬间，捕获当前的真实坐标，作为起飞的基准点
                target_x_ = current_x_.load();
                target_y_ = current_y_.load();
                target_z_ = current_z_.load();
            }

            // 【修改 2】平滑更新目标高度 (轨迹规划)
            if (offboard_setpoint_counter_ >= 10)
            {
                if (target_z_ > FINAL_Z) { 
                    target_z_ -= (ASCENT_SPEED * DT); 
                } else {
                    target_z_ = FINAL_Z; 
                }
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            if (offboard_setpoint_counter_ < 11) 
            {
                offboard_setpoint_counter_++;
            }
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    // 声明发布器和订阅器
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    uint8_t offboard_setpoint_counter_;
    uint8_t second_counter_;
    
    // 当前真实位置
    std::atomic<float> current_x_{0.0f};
    std::atomic<float> current_y_{0.0f};
    std::atomic<float> current_z_{0.0f};

    // 【修改点引入的控制变量】
    float target_x_{0.0f};
    float target_y_{0.0f};
    float target_z_{0.0f};
    const float FINAL_Z = -5.0f;     
    const float ASCENT_SPEED = 0.5f; 
    const float DT = 0.1f;           

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
    msg.position = true;                                                // 维持位置控制为主
    msg.velocity = true;                                                // 【重要修改】开启速度控制，允许接收速度前馈
    msg.acceleration = false;                                           
    msg.attitude = false;                                               
    msg.body_rate = false;                                              
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;      

    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    
    // 【修改 3】下发平滑规划的位置
    msg.position = {target_x_, target_y_, target_z_};
    
    // 下发速度前馈
    if (target_z_ > FINAL_Z) {
        msg.velocity = {0.0f, 0.0f, -ASCENT_SPEED}; 
    } else {
        msg.velocity = {0.0f, 0.0f, 0.0f};
    }

    msg.yaw = 3.14 / 2; // 维持初始航向
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
}

int main(int argc, char *argv[])
{
    std::cout << "Starting smooth offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}
