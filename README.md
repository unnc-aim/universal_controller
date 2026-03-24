# Universal Controller Framework

通用控制器框架，支持步兵和哨兵机器人的统一控制架构。

## 架构概览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Input Sources                                      │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │VTM遥控器    │ │NDJ遥控器    │ │键盘         │ │导航/行为树   │           │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘           │
└─────────┼───────────────┼───────────────┼───────────────┼──────────────────┘
          │               │               │               │
          └───────────────┴───────┬───────┴───────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │      RC Interpreter       │
                    │  发布 UnifiedInputMsg      │
                    └─────────────┬─────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │           HUB             │
                    │   模式仲裁 + 指令分发       │
                    └──────┬──────┬──────┬──────┘
                           │      │      │
          ┌────────────────┘      │      └────────────────┐
          ▼                       ▼                       ▼
   ChassisController      GimbalController       FireController
          │                       │                       │
          └───────────────────────┼───────────────────────┘
                                  ▼
                          EtherCAT Topics
```

## ROS2 话题接口

### 订阅 (Subscribers)

| Topic | Msg Type | Source | Purpose |
|-------|----------|--------|---------|
| `/ecat/sn*/app*/read` | ReadDJIMotor/ReadLkMotor/ReadDJIRC | EtherCAT | 电机反馈、遥控器 |
| `/sp_vision/autoaim_command` | AutoAimCommandMsg | 视觉 | 自瞄指令 |
| `/referee/constraints` | Float32MultiArray | 裁判系统 | 功率/热量约束 |
| `/referee/game_status` | String (JSON) | 裁判系统 | 比赛状态 |
| `/universal_controller/unified_input` | UnifiedInputMsg | RC Interpreter | 统一输入 |

### 发布 (Publishers)

| Topic | Msg Type | Purpose |
|-------|----------|---------|
| `/ecat/sn*/app*/write` | WriteDJIMotor | DJI 电机指令 |
| `/ecat/sn*/app*/write` | WriteLkMotorTorqueControl | LK 电机指令 |
| `/universal_controller/autoaim_enable` | Bool | 自瞄使能反馈 |

### Action Servers

| Action | Purpose |
|--------|---------|
| `/universal_controller/gimbal_control` | 云台控制（导航/行为树） |
| `/universal_controller/fire_control` | 发射控制（导航/行为树） |

## 消息定义

### UnifiedInputMsg

统一输入消息格式，所有输入源都转换为此格式：

```
uint8 control_source      # 0=RC, 1=KEYBOARD, 2=NAVIGATION
bool emergency_stop       # 急停标志
bool autoaim_enabled      # 自瞄使能
bool navigation_enabled   # 导航模式

# 底盘指令
float64 vx                # X方向速度 (云台系)
float64 vy                # Y方向速度 (云台系)
float64 wz                # 旋转角速度
bool spin_mode            # 小陀螺模式
float64 spin_speed        # 小陀螺速度

# 云台指令
float64 pitch_delta       # Pitch 增量 (度)
float64 yaw_delta         # Yaw 增量 (弧度)
float64 target_pitch      # 目标 Pitch (度)
float64 target_yaw        # 目标 Yaw (弧度)

# 发射指令
bool fire_trigger         # 发射触发
bool burst_mode           # 连发模式
bool friction_on          # 摩擦轮开关
```

## 模式仲裁

控制模式按以下优先级切换：

1. **EMERGENCY_STOP** (最高)
   - 遥控器离线
   - 左拨杆下档
   - 所有电机停止

2. **NAVIGATION**
   - 导航/行为树 Action 激活
   - 右拨杆中/上档

3. **AUTOAIM**
   - 左拨杆上档 + 自瞄有效

4. **MANUAL** (默认)

## 配置

所有配置集中在 `config/controller_params.yaml`：

```yaml
universal_controller:
  ros__parameters:
    control_frequency: 1000
    motor_type:
      chassis: 'DJI'
      gimbal_pitch: 'DJI'
      gimbal_yaw: 'LK'
    chassis:
      wheel_track: 0.372
      wheel_base: 0.372
      ecd_zeros: [7411, 7530, 3419, 2051]
    gimbal:
      yaw_pos_kp: 20.0
      yaw_spd_kp: 220.0
    fire:
      friction_speed_default: 6500
```

## 电机类型支持

| 组件 | DJI 电机 | LK 电机 |
|------|----------|---------|
| 底盘舵向 | 位置模式 | 级联 PID |
| 底盘驱动 | 速度模式 | 速度 PID |
| 云台 Pitch | 位置模式 | - |
| 云台 Yaw | - | 级联 PID |
| 拨盘 | 串级 PID | - |

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select universal_controller
source install/setup.bash
```

## 运行

```bash
ros2 launch universal_controller universal_controller.launch.py
```

## 文件结构

```
universal_controller/
├── CMakeLists.txt
├── package.xml
├── config/controller_params.yaml
├── launch/universal_controller.launch.py
├── msg/UnifiedInput.msg
├── action/GimbalControl.action, FireControl.action
├── include/universal_controller/
│   ├── common/types.hpp, config.hpp
│   ├── controllers/*.hpp
│   ├── interpreters/*.hpp
│   ├── hub/hub.hpp
│   └── tools/pid.hpp, swerve_kinematics.hpp
└── src/
    ├── main.cpp
    ├── hub/hub.cpp
    ├── controllers/*.cpp
    └── tools/*.cpp
```
