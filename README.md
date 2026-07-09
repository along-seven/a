# 00 — origincar Project Overview
This is a complete software system for an autonomous driving car designed for ROS2 robotics competitions. The vehicle is tasked with three core missions:
- Drive autonomously by following pre-marked lines on the ground (line following)
- Detect traffic cones and perform automatic obstacle avoidance
- Identify QR codes, pull over and wait for remote control to switch task modes

The entire system runs on the Horizon Robotics X5 embedded computing board (similar to Raspberry Pi with dedicated AI acceleration chip), which communicates with an STM32 microcontroller via serial port to control drive motors.

## Project Hardware Architecture
```
┌─────────────────────────────────────────┐
│ Horizon Robotics X3 Main Controller (ROS2 Runtime)
│ - USB Camera (visual perception)
│ - Wi-Fi (network connection & remote operation)
└────────────┬────────────────────────────┘
             │ USB Serial Port (Baud rate: 115200)
┌────────────▼────────────────────────────┐
│ STM32 Lower Computer (Microcontroller Unit)
│ - MPU6050 IMU (Gyroscope + Accelerometer for pose measurement)
│ - Wheel Encoders (odometry distance calculation)
│ - Motor Driver Module (wheel motion control)
│ - Battery Voltage Monitoring
└─────────────────────────────────────────┘
```
**Role Division**:
ROS2 framework handles high-level decision making and logic planning; STM32 executes low-level hardware control and physical sensor data acquisition.

## 2.4 Coordinate Frame Definition
ROS uses the concept of **frame** to define coordinate systems:

| Frame Name | Description | Fixed Mount Position |
|------------|-------------|----------------------|
| base_footprint | Ground projection point of the vehicle center | Ground projection of the midpoint between two driving wheels |
| base_link | Main body origin of the car | Geometric center of the chassis |
| odom_combined | World origin for fused odometry | Initial position of the vehicle upon startup |
| gyro_link | IMU sensor coordinate frame | Physical mounting position of the MPU6050 |
| laser | LiDAR coordinate frame | Mounting position of the LiDAR sensor |
| camera | Camera coordinate frame | Mounting position of the USB camera |

## 3.1 System Startup Sequence
1. Launch Chassis Driver → `origincar_base`
   - Read feedback data from STM32
   - Publish raw odometry and IMU messages
2. Launch AI Perception Module → `racing_obstacle_detection_yolo`
   - Run YOLO object detection for guide lines and traffic cones
   - Publish detection results as `PerceptionTargets` messages
3. Launch Competition Decision Control → `origincar_competition` + `qr_decoder`
   - Receive AI perception outputs
   - Execute state machine decision logic
   - Publish velocity commands to topic `cmd_vel`

Data Flow Linkage:
All upstream modules feed data into `origincar_base`. The chassis node subscribes to `cmd_vel`, encodes motion instructions into serial frames, and transmits them to STM32 to actuate wheel motors.

## 3.2 Full Data Stream Pipeline
```
USB Camera (480×272 NV12 raw frame)
        ↓
/hobot_image_raw (Original Image Topic)
        ↓ hobot_codec hardware encoding (zero-copy shared memory)
/hbmem_img (Shared memory NV12 image buffer)
        ↓ BPU hardware-accelerated YOLOv5 DNN inference
Perception Detection Node
        ↓ Publish target bounding boxes & classes → /origincar_competition
```

QR Code Recognition Branch:
```
/hobot_image_raw → ZBar QR Decoder Node (/qr_decoder)
        ↓ Publish decoded QR string → /sign topic
```

Central Decision & Actuation Flow:
```
Perception Target Topic + QR Sign Topic
        ↓ Input to complete_control_node (state machine core)
        ↓ Output linear & angular velocity → /cmd_vel (Twist message)
        ↓ origincar_base serial packaging & transmission
        ↓ STM32 motor execution
STM32 feeds back encoder + IMU data
        ↓ origincar_base publishes /odom and /imu/data_raw
        ↓ EKF Extended Kalman Filter sensor fusion
        ↓ Final fused odometry output → /odom_combined
```

## Module Function List
| Index | Package Name | Core Function |
|-------|--------------|---------------|
| 01 | origincar_msg | Custom ROS message definitions (custom data envelope format) |
| 02 | origincar_description | URDF model description: vehicle 3D structure, dimension parameters |
| 03 | origincar_base | Chassis management module: serial communication with STM32, raw odometry & IMU calculation |
| 04 | origincar_bringup | Launch entry manager: one-click startup for camera, encoding and AI inference pipeline |
| 05 | origincar_competition | Competition decision brain: line following logic, cone avoidance, finite state machine control |
| 06 | qr_decoder | QR code recognition module: parse QR content from image frames |
| 07 | racing_obstacle_detection_yolo | AI vision module: YOLOv5 object detection for guide lines and traffic cones via BPU acceleration |
| 08 | image_upload_analyzer | Remote cloud debugging tool: upload camera frames to cloud server for offline analysis |
| 09 | utils | Format conversion toolkit: convert NV12 image format to standard ROS sensor_msgs/Image |
| 10 | 3rdparty | Third-party dependency library: serial port driver, Ackermann steering message definitions, depth camera drivers |

## Core Control Topic Communication Table
### Main Motion Control Link
| Topic Name | Message Type | Publisher | Subscriber | Trigger Condition |
|------------|-------------|-----------|------------|-------------------|
| /origincar_competition | PerceptionTargets | YOLO Detection Node | Competition Control Node | Published every frame after object inference completes |
| /cmd_vel | geometry_msgs/Twist | Competition Control Node | origincar_base Chassis Node | Published once per motion decision |
| /odom | nav_msgs/Odometry | origincar_base Chassis Node | EKF Fusion Node | Updated upon receiving new serial data from STM32 |
| /imu/data_raw | sensor_msgs/Imu | origincar_base Chassis Node | EKF Fusion + Madgwick Filter | Updated upon receiving new serial data from STM32 |
| /odom_combined | nav_msgs/Odometry | EKF Fusion Node | Navigation & external positioning modules | Continuously output fused high-precision odometry |

### Task Switching Link
| Topic Name | Message Type | Publisher | Subscriber | Trigger Condition |
|------------|-------------|-----------|------------|-------------------|
| /sign | std_msgs/String | QR Decoder Node | Competition Control Node | Published immediately when a valid QR code is scanned |
| /sign4return | std_msgs/Int32 | Remote Operation Node | Competition Control Node | Remote command to enable/disable manual override |
| /sign_switch | Custom Sign Msg | Competition Control Node | Upper Monitoring PC | Feedback task state after QR code trigger |
| /PowerVoltage | std_msgs/Float32 | origincar_base Chassis Node | System Monitor Node | Battery voltage published every 10 serial data frames |

## End-to-End Workflow: Cone Detection & Obstacle Avoidance
Scenario: Vehicle is running line-following mode and a traffic cone appears in the forward field of view
1. USB camera captures the scene frame, raw image is published to `/hobot_image_raw`
2. hobot_codec encodes the image into shared memory buffer `/hbmem_img` for zero-copy transmission
3. YOLO detection module runs BPU inference and identifies two targets:
   - Class `line` (guide line): Bounding box coordinate (x=300, y=100, width=40, height=200)
   - Class `zt` (traffic cone): Bounding box coordinate (x=320, y=50, width=60, height=190)
4. Detection results are encapsulated into `PerceptionTargets` message and sent to `/origincar_competition`
5. Competition decision node processes data:
   - Cone bounding box height 190px exceeds threshold `180px` → Enter `CONE_AVOIDANCE_ACTIVE` state
   - Calculate horizontal offset between cone and guide line: `320 - 300 = 20`, meaning the cone is directly ahead
   - Output velocity command: linear.x = 0.3 m/s, angular.z = 2.0 rad/s (steer left to bypass obstacle)
6. `cmd_vel` Twist message is subscribed by the chassis driver node
7. origincar_base encodes velocity command into serial protocol frames and sends to STM32
8. STM32 adjusts PWM duty cycle for left/right motors: slow left wheel, speed up right wheel, vehicle turns left to avoid the cone
9. Once the cone exits the camera field of view, the state machine switches to line searching mode
10. Guide line is re-detected, system reverts to normal line-following state

## Parameter Modification Quick Reference
| Target Parameter | File Path | Specific Config Item |
|------------------|-----------|----------------------|
| Default line following speed | origincar_competition/launch/start.launch.py | `line_following_speed` |
| Line correction PID proportional gain | origincar_competition/launch/start.launch.py | `line_kp` |
| Minimum cone height threshold to trigger avoidance | origincar_competition/launch/start.launch.py | `cone_detection_y_threshold` |
| Vehicle speed during cone avoidance | origincar_competition/launch/start.launch.py | `cone_avoidance_speed` |
| Steering gain during obstacle avoidance | origincar_competition/launch/start.launch.py | `cone_avoidance_steering_gain` |
| Serial port device name for STM32 communication | origincar_base/src/origincar_base.cpp (Class Constructor) | Argument inside `setPort()` function |
| Chassis wheel track & wheelbase | origincar_description/urdf/origincar.xacro | `Track` (wheel spacing), `WheelBase` (front-rear axle distance) |
| Custom action after QR code scan | origincar_competition/src/origincar_competition.cpp | Callback function `qrCodeRawCallback()` |
| Add new detectable object classes | racing_obstacle_detection_yolo/config/obstacles.list | Append new class name entries |

## Compilation & Runtime Instructions
```bash
# Enter ROS2 colcon workspace
cd ~/dev_ws

# Build all packages in the workspace
colcon build

# Source environment variables to register compiled packages
source install/setup.bash

# Launch full competition autonomous task program
ros2 launch origincar_competition start.launch.py
```
