# Offboard Demo

一个基于 PX4 和 ROS 2 的 Offboard 控制功能包，使用 C++ 编写，包含多个演示节点。

## 系统要求

| 组件 | 版本 |
| :--- | :--- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 | 1.16.1 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |

## 项目包含

本功能包包含以下演示节点：

1.  **offboard_demo**: 带有状态机的 Offboard 控制演示。
2.  **offboard_demo_odometry**: 订阅 `/fmu/out/vehicle_odometry` 话题，实现基于里程计数据的高度反馈控制演示。
3.  **offboard_demo_local_position**: 订阅 `/fmu/out/vehicle_local_position` 话题，实现基于本地位置数据的高度反馈控制演示。
4.  **offboard_demo_autoland**: 订阅 `/fmu/out/vehicle_local_position` 话题，实现自动降落演示。

## 安装与编译

1.  **创建 ROS 2 工作空间并克隆依赖**
    ```bash
    mkdir -p ~/ws_ros2/src/
    cd ~/ws_ros2/src/
    # 克隆 PX4 消息接口与 ROS 2 桥接包（分支对应 PX4 1.16.1）
    git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
    git clone -b release/1.16 https://github.com/PX4/px4_ros_com.git
    # 克隆本功能包
    git clone https://github.com/4Shuan9/offboard_demo.git
    ```

2.  **编译工作空间**
    ```bash
    cd ~/ws_ros2
    source /opt/ros/humble/setup.bash
    colcon build
    ```

3.  **设置环境变量**
    ```bash
    # 将环境设置命令添加到 .bashrc（仅需执行一次）
    echo "source ~/ws_ros2/install/setup.bash" >> ~/.bashrc
    # 立即生效
    source ~/.bashrc
    ```

## 使用

编译并设置好环境后，可以使用 `ros2 run` 命令运行各个演示节点。例如，运行自动降落演示：

```bash
ros2 run offboard_demo offboard_demo_autoland
