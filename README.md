# Offboard Demo

An Offboard control package based on PX4 and ROS 2, written in C++, containing multiple demo nodes.

## System Requirements

| Component | Version |
| :--- | :--- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 | 1.16.1 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |

## Package Contents

This package includes the following demo nodes:

1. **offboard_demo**: Offboard control demo with a state machine.
2. **offboard_demo_odometry**: Subscribes to `/fmu/out/vehicle_odometry` topic and implements altitude feedback control based on odometry data.
3. **offboard_demo_local_position**: Subscribes to `/fmu/out/vehicle_local_position` topic and implements altitude feedback control based on local position data.
4. **offboard_demo_autoland**: Subscribes to `/fmu/out/vehicle_local_position` topic and implements automatic landing demo.

## Installation & Build

1. **Create a ROS 2 workspace and clone dependencies**
    ```bash
    mkdir -p ~/ws_ros2/src/
    cd ~/ws_ros2/src/
    # Clone PX4 message interfaces and ROS 2 bridge package (branch matching PX4 1.16.1)
    git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
    git clone -b release/1.16 https://github.com/PX4/px4_ros_com.git
    # Clone this package
    git clone https://github.com/4Shuan9/offboard_demo.git
    ```

2. **Build the workspace**
    ```bash
    cd ~/ws_ros2
    source /opt/ros/humble/setup.bash
    colcon build
    ```

3. **Set up environment variables**
    ```bash
    # Add the environment setup command to .bashrc (only needs to be done once)
    echo "source ~/ws_ros2/install/setup.bash" >> ~/.bashrc
    # Apply immediately
    source ~/.bashrc
    ```

## Usage

After building and setting up the environment, you can run each demo node using the `ros2 run` command. For example, to run the automatic landing demo:

```bash
ros2 run offboard_demo offboard_demo_autoland
