# IO 层完整信息流：从上到下

## 一、有多少个 IO 通道？

从 [config_sentry.hpp](../GKD_Control/include/configs/config_sentry.hpp) 的配置：

```
io<CAN>      3 条 CAN 总线:
             ├── "CAN_CHASSIS"    底盘 CAN（4 个轮子电机 + 超级电容）
             ├── "CAN_GIMBAL"     云台 CAN（yaw 电机 + pitch 电机 + 摩擦轮 ×2）
             └── "CAN_BULLET"     发射 CAN（拨弹电机）

io<SERIAL>   2 个串口:
             ├── "/dev/IMU_SMALL_YAW"    小型 IMU
             └── "/dev/IMU_BIG_YAW"      大型 IMU（也承载遥控器数据）

io<SOCKET>   1 个 UDP Socket:
             └── "AUTO_AIM_CONTROL"      视觉自瞄（端口 11451）
```

## 二、有一个总 IO 吗？

**没有**。三个是平级的全局变量，各管各的：

```cpp
// 来自同一个变量模板，但独立实例化
IO<CAN>    io<CAN>;       // 只管理 CAN 设备
IO<SERIAL> io<SERIAL>;    // 只管理 Serial 设备
IO<SOCKET> io<SOCKET>;    // 只管理 Socket 设备
```

不是"一个总管三个子"，而是三个**独立的字典**。想找 CAN 就去 `io<CAN>`，想找串口就去 `io<SERIAL>`。

## 三、信息流完整路径（5 步）

### 第 1 步：load_hardware() — 创建 IO 对象 + 启动 task() 线程

```cpp
// robot_controller.cc:63-73
void Robot_ctrl::load_hardware() {
    // 对配置里的每个名字，创建 Can_interface 对象，启动它的 task() 线程
    for (auto& name : Config::CanInitList) {
        IO::io<CAN>.insert(name);
    }
    // 同理创建串口
    for (auto& [name, baud, timeout] : Config::SerialInitList) {
        IO::io<SERIAL>.insert(name, baud, timeout);
    }
    // 同理创建 Socket
    for (auto& name : Config::SocketInitList) {
        IO::io<SOCKET>.insert(name);
    }
}
// 此时：3 CAN + 2 Serial + 1 Socket 的 task() 线程都在跑
// 但还没有人注册回调 → 收到的数据没人处理
```

### 第 2 步：start_init() — 设备向 IO 注册回调

```cpp
// robot_controller.cc:24-36
void Robot_ctrl::start_init() {
    chassis.init(robot_set);
    //    └→ 构造 4 个 DJIMotor("CAN_CHASSIS", id=1,2,3,4)
    //         └→ motor.enable()
    //              └→ DJIMotorManager::register_motor()
    //                   ├→ can = IO::io<CAN>["CAN_CHASSIS"]     // 找到 CAN 通道
    //                   └→ can->register_callback_key(0x201, lambda) // 注册回调

    gimbal.init(robot_set);
    //    └→ yaw_motor("CAN_GIMBAL", id=2).enable()
    //         └→ can = IO::io<CAN>["CAN_GIMBAL"]
    //              └→ can->register_callback_key(0x206, lambda)
    //    └→ IMU::enable()
    //         └→ serial = IO::io<SERIAL>["/dev/IMU_SMALL_YAW"]
    //              └→ serial->register_callback<IMU包>(lambda)

    referee.init(robot_set);
    //    └→ 裁判系统用自己独立的串口线程读数据
}
```

### 第 3 步：运行时 — 数据从硬件流入

```
物理 CAN 总线上来一帧: can_id=0x201, data=[角度高,角度低,转速高,转速低,电流高,电流低,温度,保留]
      │
      ▼ Linux 内核 SocketCAN 驱动，放入 socket 缓冲区
      │
      ▼ Can_interface::task()  ← 独立的 IO 线程
      │   read(socket, &frame, sizeof(can_frame))
      │
      ▼ callback_key(0x201, frame)  ← 继承自 Callback_key 的查表方法
      │
      ▼ 查 map: {0x201: lambda1, 0x202: lambda2, ...}
      │
      ▼ lambda1(frame)  ← DJIMotorManager::register_motor 里注册的
      │
      ▼ motor.unpack(frame)
          ├→ data_.rotor_angle  = ECD × (2π/8192)    // 编码器 → 弧度
          ├→ data_.speed_rpm    = 转速                 // RPM
          └→ update_time()                             // 标记在线
```

### 第 4 步：控制模块读取（另一个独立线程）

```cpp
// Chassis::task()  500Hz
while (true) {
    // 读第3步更新的电机反馈数据
    float actual_speed = motor.data_.output_linear_velocity;

    // 做 PID 控制
    wheel_speed[i] >> wheel_pid >> motor.set(x);
    //                                   │
    //                              motor.give_current = x  ← 存下电流指令

    sleep_ms(2);
}
```

### 第 5 步：数据流出 — 打包发送到硬件

```cpp
// DJIMotorManager::task()  1000Hz
while (true) {
    for (auto& [can_name, can_block] : motors_map) {
        // 收集每条 CAN 上 4 个电机的 give_current（第4步写的）
        can_frame frame = {.can_id = 0x200, .len = 8};
        for (auto motor : can_block.motors_) {
            frame.data[motor->data_bias]   = motor->give_current >> 8;  // 高字节
            frame.data[motor->data_bias|1] = motor->give_current & 0xff; // 低字节
        }
        can_block.can_->send(frame);
        //              │
        //              └→ write(socket, frame) → Linux SocketCAN → 物理总线 → 电机执行
    }
    sleep_until(now + 1ms);
}
```

## 四、CAN 帧的完整闭环

```
                    ┌──────────────────────────────────────┐
                    │          DJIMotorManager              │
                    │                                      │
   ┌────────────────┤  task() 1kHz:                        │
   │                │  打包 4 个电机的 give_current          │
   │                │  → can->send(frame)                  │
   │ 电流指令        └──────────────────┬───────────────────┘
   │ 到电机                              │
   ▼                                    ▼
┌──────┐                          ┌──────────┐
│ 电机  │ ── CAN 帧 (0x201) ──→   │  CAN 总线  │
└──────┘  角度/转速/温度/电流       └────┬─────┘
                                        │
                                        ▼
                              ┌──────────────────────────┐
                              │ Can_interface::task()     │
                              │ read() → callback_key()   │
                              └────────┬─────────────────┘
                                       │
                                       ▼
                              ┌──────────────────────────┐
                              │ motor.unpack(frame)       │
                              │ 更新 data_.rotor_angle    │
                              │ 更新 data_.speed_rpm      │
                              └────────┬─────────────────┘
                                       │
                                       ▼
                              ┌──────────────────────────┐
                              │ Chassis/Gimbal::task()    │
                              │ 读 motor.data_ → PID →    │
                              │ motor.set(x) →            │
                              │ motor.give_current = x    │
                              └────────┬─────────────────┘
                                       │
                                       └────→ 回到 DJIMotorManager，形成闭环
```

## 五、对比：CAN vs Serial vs Socket

| | CAN | Serial | Socket |
|---|---|---|---|
| 接口对象 | `Can_interface` | `Serial_interface` | `Server_socket_interface` |
| 物理层 | CAN 总线 | UART 串口 | UDP/IP |
| 全局字典 | `io<CAN>` | `io<SERIAL>` | `io<SOCKET>` |
| 回调机制 | `Callback_key<id, frame>` | `Callback<IMU包, RC包>` | `Callback_key<header, data>` + `Callback<导航>` |
| 按什么找 | CAN ID（整数） | C++ 类型 | header 字节 |
| 谁注册 | 电机、超级电容 | IMU、遥控器 | 云台（自瞄）、导航 |
| 方向 | **双向**（收反馈+发指令） | **收为主** | **双向**（收目标+发当前角度） |
