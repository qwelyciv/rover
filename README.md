# Rovernauts — ROS2 Control Package 🚀

Code I wrote for our university rover team (**Rovernauts / LASER**, University of Liverpool) competing in the **UKSEDS Olympus Rover Trials 2025/26** (Advanced Stream). 

We made it through PDR and reached the Critical Design Review (CDR) stage with an 86% score.

## What is this?
Basically the brain of the rover running on a **Raspberry Pi 4 (ROS2 Humble)**. Because a single Pi shouldn't be busy trying to generate tight PWM and step pulses while running SLAM/video, we split the hardware:
* **Raspberry Pi 4**: runs ROS2, web server, video feed, and these Python nodes.
* **ESP32**: handles the 4 DC drive motors (Cytron MDD10A drivers) and the front rack-and-pinion steering stepper. Connected via USB serial.
* **Raspberry Pi Pico**: runs MicroPython to drive the 4 stepper motors on the 3D-printed arm. Also connected via USB serial.

## The Nodes
* **`drive_bridge.py`** — Takes `/cmd_vel` from either the operator (web UI) or Nav2 autonomous navigation. RC input always overrides autonomous mode so we don't crash into rocks. It converts linear/angular velocity into actual steering angles using a bicycle model ($L = 20\text{ cm}$), turns that into stepper steps for the 16-tooth rack gear, and sends JSON packets to the ESP32. Has a 0.5s watchdog so the rover stops dead if Wi-Fi drops.
* **`arm.py`** — Talks to the Pico to move the 4-DOF digging arm (uses modded 28BYJ-48 5V steppers). Has software limits so the links don't smash into the rover frame, plus automated sequences (`STORE` to dig sand into the container, `DEPOSIT` to dump it, and `STOW` to fold the arm flat so the rover fits the size limits for the vibration test).
* **`telemetry.py`** — Simple async WebSocket server (port 8765) for our web dashboard. Pushes Pi CPU usage, temperature, Wi-Fi signal strength (dBm from the external TP-Link dongle), and IMU tilt angles every second so we know if the rover is about to tip over (>25° warning).

## Setup & Running

Install the required Python packages first:
```bash
pip3 install pyserial psutil websockets

Build the package inside your ROS2 workspace:

Bash
cd ~/ros2_ws
colcon build --packages-select rovernauts_control
source install/setup.bash

Run whatever node you need:

Bash
ros2 run rovernauts_control drive_bridge
ros2 run rovernauts_control arm
ros2 run rovernauts_control telemetry
