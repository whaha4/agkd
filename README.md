# 相对 c634f18 的修改说明

本文只记录本仓库相对 `c634f18` 的代码修改，不包含项目构建与整体架构说明。

- 对比基线：`c634f18`（2026-07-04）
- 当前代码：`0e0bbcf`（分支 `回退超电通信超时降级，保留其余功率修复`）
- 代码差异：15 个文件，+183 / −112（不含 `may_imporve/` 下的方案文档）

本次修改分为两部分：**单轮离线容错**、**PID 条件积分抗饱和**；其余改动统一归入“其他修复”。

## 修改文件

| 文件 | 修改内容 |
| --- | --- |
| `GKD_Control/src/chassis/chassis.cc` | 轮速 / 角速度量纲统一；离线轮处理；底盘输出改用功率限制后的电流 |
| `GKD_Control/include/chassis/power_controller.hpp` | 注释中的转速 / 扭矩量纲改为输出轴 |
| `GKD_Control/src/chassis/power_controller.cc` | 功率模型量纲统一、NaN 防护、日志限频、离线轮功率处理 |
| `GKD_Control/src/device/super_cap.cc` | 修正 0x61 控制帧字段顺序 |
| `GKD_Control/include/device/deviece_base.hpp` | 离线计时改用 `steady_clock`；新增 `set_offline_time()` |
| `GKD_Control/src/device/device_base.cc` | 同上，实现离线时间设置 |
| `GKD_Control/src/control/pid_controller.cc` | `PidPosition`、`PidRad` 加入条件积分抗饱和 |
| `ControlLib/src/pid.cpp` | 同步加入条件积分抗饱和 |
| `GKD_Control/src/shoot/shoot.cc` | 堵弹时清理拨弹 PID 状态 |
| `GKD_Control/src/control/controller.cc` | 修复 `ControllerLirelost` 拼写错误 |
| `GKD_Control/include/robot_controller/robot_controller.hpp` | 修复 `std::vectorstd::jthread>` 拼写错误 |
| `GKD_Control/xmake.lua` | 显式加入 `add_files("src/*.cc", "src/**/*.cc")` |
| `hello.cpp`、`tests.cpp`、`tests.exe` | 删除根目录临时文件 |

## 1. 单轮离线容错

`GKD_Control/src/device/device_base.cc`、`GKD_Control/include/device/deviece_base.hpp`、`GKD_Control/src/chassis/chassis.cc`、`GKD_Control/src/chassis/power_controller.cc`

- `DeviceBase` 从 `system_clock` 迁移到 `steady_clock`，避免系统时间跳变影响离线判断；新增 `set_offline_time()`。
- 轮电机离线判定改为 40 ms（C620 约 1 kHz 反馈，约等于连续丢 40 帧）。
- 离线轮：
  - 不参与最大速度统计；
  - 每个控制周期调用 `wheels_pid[i].clean()`，恢复时不会带旧积分；
  - 不参与功率分配，`curAv` / `pidOutput` / `setAv` / `pidMaxOutput` 置 0；
  - 下发电流置 0。
- 功率管理为离线轮预留 `k3/4` 固定损耗，但不把离线轮计入需求、误差权重和 RLS 统计，并置 `MotorDisconnect` 错误标志。
- 其余三轮继续工作，实现“掉一个轮子还能跑”。

## 2. PID 条件积分抗饱和

`GKD_Control/src/control/pid_controller.cc`、`ControlLib/src/pid.cpp`

`PidPosition` 和 `PidRad` 在积分前判断：输出已饱和且误差仍在往饱和方向推时，本拍不累加积分。以 `PidPosition` 为例：

```cpp
const fp32 pd = Pout + Dout;
const fp32 unsat = pd + Iout;

// 条件积分: 输出已饱和且误差仍在往饱和方向推时，本拍不再积分
if (!(std::abs(unsat) > max_out && error[0] * unsat > 0.f)) {
    Iout += ki * error[0];
}
Iout = std::clamp(Iout, -max_iout, max_iout);

out = std::clamp(pd + Iout, -max_out, max_out);
```

- 未饱和时行为与原实现完全一致；
- 饱和且误差继续同向推时停止积分，输出仍照常限幅；
- `GKD_Control` 与 `ControlLib` 两份 PID 实现同步修改。

## 3. 其他修复

### 3.1 底盘功率模型量纲统一

`GKD_Control/src/chassis/power_controller.cc`、`GKD_Control/src/chassis/chassis.cc`

- `curAv` 从 `speed_rpm * π / 30` 改为电机输出轴角速度 `output_angular_velocity`。
- `setAv` 从轮速（m/s）改为 `wheel_speed / radius`（输出轴 rad/s）。
- `k0` 去掉转子到输出轴的减速比折算，改为 `0.3 × 20 / 16384`（输出轴 N·m / 电流指令单位）。
- 功率模型四项、RLS 输入、`estimatedPower` 全部统一到输出轴量纲：

  ```
  P ≈ τ·ω + k1·|ω| + k2·τ² + k3/4
  ```

- 底盘实际下发从原始 PID 输出改为功率限制后的 `cmd_power[i]`。

### 3.2 超电控制帧字段修正

`GKD_Control/src/device/super_cap.cc`

- 修正 0x61 控制帧中 `chassis_power_limit` 与 `chassis_power_buffer` 两个字段写反的问题。

### 3.3 数值与日志健壮性

`GKD_Control/src/chassis/power_controller.cc`

- `newTorqueCurrent` 默认初始化为 0，避免异常分支留下未初始化值。
- `sumError` / `sumPowerRequired` 接近 0 时不再直接相除，避免 NaN 电流指令。
- `std::clamp` 前先对 `fullMaxPower` / `baseMaxPower` 排序，避免上下界颠倒。
- `powerStatus.efficiency` 在 `measuredPower` 接近 0 时直接取 0，避免除零。
- 功率日志从每 2 ms 输出改为 500 ms 限频，避免热路径阻塞。

### 3.4 堵弹时清理拨弹控制器

`GKD_Control/src/shoot/shoot.cc`

- 堵弹判定仍为 `given_current > 4000 && speed_rpm < 1`。
- 堵弹后拨弹输出置零，并调用 `trigger.controller.clean()` 清掉积分 / 误差，防止解除后带着旧输出。

### 3.5 编译与清理

- 修复 `ControllerList` 的 `ControllerLirelost` 拼写错误。
- 修复 `std::vectorstd::jthread>` 拼写错误。
- xmake 显式加入 `add_files`。
- 删除根目录 `hello.cpp`、`tests.cpp`、`tests.exe`。

## 验证

- PID：离线数值对比，未饱和场景新旧输出一致；持续堵转 500 拍时新实现积分保持 0，旧实现达到积分上限。
- 整机：尚未在 Linux 下完整构建，也未实车验证；上车前需完整编译并做台架测试。
