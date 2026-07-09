# 00 — origincar 项目总体概览（

---



这是一个**自动驾驶小车**的完整软件系统，用在 ROS2 机器人竞赛中。小车的任务是：

1. **沿着地上的线自动行驶**（巡线）
2. **看到锥桶自动绕开**（避障）
3. **识别二维码并停车等待遥控**（任务切换）

整个系统跑在一台叫"地平线 X5"的嵌入式小电脑上（类似树莓派但有 AI 加速芯片），通过串口线连接一个 STM32 单片机来控制电机。

---


本项目的硬件

```
┌─────────────────────────────────────────┐
│     地平线 X3 主控板（运行ROS2）          │
│     - USB摄像头（看路）                   │
│     - WiFi（连网/遥操作）                 │
└────────────┬────────────────────────────┘
             │ USB串口 (115200波特率)
┌────────────▼────────────────────────────┐
│     STM32 下位机（单片机）                │
│     - MPU6050 陀螺仪+加速度计（测姿态）    │
│     - 轮子编码器（测走了多远）             │
│     - 电机驱动（控制轮子转）               │
│     - 电池电压检测                        │
└─────────────────────────────────────────┘
```

ROS2 这边负责"思考和决策"，STM32 那边负责"执行和感知物理世界"。

### 2.4 坐标系说明

ROS 中用"frame"来表示坐标系：

| 坐标系名 | 含义 | 固定在哪 |
|----------|------|----------|
| `base_footprint` | 小车在地面上的投影点 | 两轮中心地面 |
| `base_link` | 小车本体中心 | 底盘几何中心 |
| `odom_combined` | 里程计世界原点 | 小车启动时的位置 |
| `gyro_link` | IMU传感器位置 | 陀螺仪安装位置 |
| `laser` | 激光雷达位置 | 激光安装位置 |
| `camera` | 摄像头位置 | 摄像头安装位置 |

---



### 3.1 启动流程

```
第一步：启动底盘驱动       第二步：启动AI感知        第三步：启动竞赛控制
   (origincar_base)    (racing_obstacle    (origincar_competition
                        _detection_yolo)     + qr_decoder)
        │                      │                      │
  读取STM32数据         YOLO检测线和锥桶        接收AI结果→决策→发cmd_vel
  发布里程计/IMU        发布PerceptionTargets    发到cmd_vel topic
        │                      │                      │
        └──────────────────────┴──────────────────────┘
                               │
                     origincar_base 收到 cmd_vel
                       编码成串口数据发STM32
                        STM32控制电机转动
```

### 3.2 数据流全景图

```
USB摄像头              hobot_codec编码         DNN YOLO推理
  │480×272 NV12         │共享内存NV12           │BPU硬件加速
  ▼                     ▼                      ▼
/image ──────────→ /hbmem_img ──────────→ /origincar_competition
(NV12编码)        (共享内存零拷贝)        (检测到的线和锥桶坐标)
                                                 │
                    QR码识别 ◄── /image          │
                    (ZBar库)     (JPEG)          │
                       │                         │
                       ▼                         ▼
                    /sign ────→ complete_control_node
                  (字符串)       (状态机决策)
                                     │
                                     ▼
                                 cmd_vel
                              (线速度+角速度)
                                     │
                                     ▼
                           origincar_base_node
                              (串口编码发送)
                                     │
                                     ▼
                              STM32 执行
                         (发布IMU/编码器数据)
                                     │
                                     ▼
                              /odom + /imu/data_raw
                                     │
                                     ▼
                           EKF卡尔曼滤波融合
                                     │
                                     ▼
                              odom_combined
                            (融合后的精准里程计)
```

---



| 编号 | 模块名 | 一句话 |
|------|--------|--------|
| 01 | origincar_msg | 定义项目自己的消息格式（类似自定义信封） |
| 02 | origincar_description | 描述小车长什么样（3D模型、尺寸） |
| 03 | origincar_base | **底盘大管家**：跟STM32对话、算里程计、发IMU |
| 04 | origincar_bringup | **启动总管**：一键启动摄像头+编码+推理 |
| 05 | origincar_competition | **竞赛大脑**：看线、看锥桶、决策怎么走 |
| 06 | qr_decoder | **扫码器**：从图像中识别二维码内容 |
| 07 | racing_obstacle_detection_yolo | **AI眼睛**：用YOLOv5识别线和锥桶 |
| 08 | image_upload_analyzer | **远程分析师**：把图发到云端AI分析 |
| 09 | utils | **格式转换器**：NV12图像格式转标准ROS图像 |
| 10 | 3rdparty | **外援库**：串口库、阿克曼消息、深度相机驱动 |

---



### 控制链路（最核心！）

| Topic | 类型 | 谁发 | 谁收 | 什么时候发 |
|-------|------|------|------|-----------|
| `/origincar_competition` | PerceptionTargets | YOLO检测节点 | 竞赛控制节点 | 每帧图像检测完后 |
| `cmd_vel` | Twist | 竞赛控制节点 | 底盘驱动节点 | 每次决策后 |
| `odom` | Odometry | 底盘驱动节点 | EKF滤波器 | 收到STM32数据后 |
| `/imu/data_raw` | Imu | 底盘驱动节点 | EKF滤波器+Madgwick | 收到STM32数据后 |
| `odom_combined` | Odometry | EKF滤波器 | 导航等外部模块 | 融合后持续输出 |

### 任务切换链路

| Topic | 类型 | 谁发 | 谁收 | 什么时候发 |
|-------|------|------|------|-----------|
| `/sign` | String | QR解码节点 | 竞赛控制节点 | 扫到二维码 |
| `/sign4return` | Int32 | 遥操作节点 | 竞赛控制节点 | 遥控开始/结束 |
| `/sign_switch` | Sign | 竞赛控制节点 | 上位机 | QR码识别后 |
| `/PowerVoltage` | Float32 | 底盘驱动节点 | 监控节点 | 每10帧数据 |

---

## 六、一个完整的"看到锥桶→绕开"过程

假设小车正在巡线行驶，前方出现了一个锥桶：

```
1. 摄像头拍到画面 → 经编码后发到 /image
2. hobot_codec 把图像转成共享内存格式 /hbmem_img
3. YOLO检测节点收到图像，BPU推理，发现：
   - 一个 "line" 目标（引导线），坐标 (x=300, y=100, w=40, h=200)
   - 一个 "zt" 目标（锥桶），坐标 (x=320, y=50, w=60, h=190)
4. 检测结果打包成 PerceptionTargets 发到 /origincar_competition
5. 竞赛控制节点收到：
   - 读取锥桶高度 = 190px > 180px阈值 → 进入 CONE_AVOIDANCE_ACTIVE 状态
   - 计算锥桶相对于线的偏移 = 320-300 = 20 → 在正前方
   - 输出: linear.x = 0.3, angular.z = 2.0（左转避障）
6. cmd_vel 消息到达底盘驱动节点
7. 底盘节点编码成串口数据帧发给STM32
8. STM32 控制电机：左轮减速、右轮加速 → 小车向左绕开锥桶
9. 锥桶从视野消失 → 竞赛节点进入向前搜索线状态
10. 找到线 → 回到巡线状态，继续行驶
```

---

| 想改什么 | 去哪个文件 | 改哪个参数 |
|----------|-----------|-----------|
| 小车巡线速度 | origincar_competition/launch/start.launch.py | `line_following_speed` |
| 巡线纠偏灵敏度 | 同上 | `line_kp` |
| 避障触发距离 | 同上 | `cone_detection_y_threshold` |
| 避障速度 | 同上 | `cone_avoidance_speed` |
| 避障转向大小 | 同上 | `cone_avoidance_steering_gain` |
| 串口设备名 | origincar_base/src/origincar_base.cpp 构造函数 | `setPort()` 参数 |
| 轮距/轴距 | origincar_description/urdf/origincar.xacro | Track/WheelBase |
| QR码触发行为 | origincar_competition/src/origincar_competition.cpp | `qrCodeRawCallback()` |
| 要检测的新物体 | racing_obstacle_detection_yolo/config/obstacles.list | 添加类别名 |

### 编译运行流程：
```bash
cd ~/dev_ws                    # 进入你的ROS2工作空间
colcon build                   # 编译所有包
source install/setup.bash      # 加载环境
ros2 launch origincar_competition start.launch.py   # 启动竞赛模式
```
