# LASER Rovernauts — ROS2 Control & Telemetry System

Software subsystem developed for the **University of Liverpool (LASER)** entry to the **UKSEDS Olympus Rover Trials 2025–2026** (Advanced Stream)[cite: 1]. 

## System Architecture
Built on **ROS2 Humble Hawksbill (Python)** running on a **Raspberry Pi 4 Model B**, interfacing with distributed microcontrollers (**ESP32** and **Raspberry Pi Pico**) over UART/Serial[cite: 1].

### Core Nodes (`rovernauts_control`)
1. **`drive_mux_bridge`**: 
   - Implements hardware control multiplexing (`SW-03`), prioritizing operator manual RC overrides over autonomous **Nav2** velocity commands[cite: 1].
   - Converts `geometry_msgs/Twist` commands into PWM signals for dual **Cytron MDD10A** drivers (4x **FIT0186** 12V geared motors) and step targets for the 16-tooth rack-and-pinion steering stepper[cite: 1].
   - Enforces a 500ms watchdog fail-safe timeout on communication loss[cite: 1].
2. **`arm_controller`**:
   - High-level kinematic & safety controller for the 4-DOF excavation manipulator (4x bipolar **28BYJ-48** steppers driven by a **Raspberry Pi Pico**)[cite: 1].
   - Enforces software joint angle limits and executes automated asynchronous state-machine macros (`STORE`, `DEPOSIT`, and `STOW` for Airbus vibration test compliance)[cite: 1].
3. **`gcs_telemetry`**:
   - Asynchronous WebSocket bridge for Non-LOS Ground Control Station (GCS) operation[cite: 1].
   - Broadcasts 1 Hz system diagnostics (`SW-02`: SoC temperature, CPU load, **TL-WN722N** RSSI dBm) and **ICM-20948** 9-DOF IMU orientation with 30° incline stability warnings[cite: 1].

## Build & Run Instructions
```bash
# Clone into ROS2 workspace src directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone [https://github.com/](https://github.com/)<ТВОЄ_ІМ'Я_КОРИСТУВАЧА>/rovernauts-ros2-control.git

# Install dependencies and build
pip3 install pyserial psutil websockets
cd ~/ros2_ws
colcon build --packages-select rovernauts_control
source install/setup.bash

# Launch nodes
ros2 run rovernauts_control drive_mux_bridge
ros2 run rovernauts_control arm_controller
ros2 run rovernauts_control gcs_telemetry
