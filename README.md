# 相对 c634f18 的修改说明

本文只记录本仓库相对 `c634f18` 的两项修改：**单轮离线容错**、**PID 条件积分抗饱和**。

- 对比基线：`c634f18`（2026-07-04，来源于gkd仓库）
- 当前代码：`0e0bbcf`（分支 `回退超电通信超时降级，保留其余功率修复`）
- 代码差异：15 个文件，+183 / −112（下表只列与这两项修改直接相关的文件）

## 修改文件

| 文件 | 修改内容 |
| --- | --- |
| `GKD_Control/src/device/device_base.cc` | 离线计时改用 `steady_clock`；实现 `set_offline_time()` |
| `GKD_Control/include/device/deviece_base.hpp` | 新增 `set_offline_time()` 声明 |
| `GKD_Control/src/chassis/chassis.cc` | 离线轮不参与限速、清理 PID、输出置零 |
| `GKD_Control/src/chassis/power_controller.cc` | 离线轮功率预留、排除与 RLS 跳过 |
| `GKD_Control/src/control/pid_controller.cc` | `PidPosition`、`PidRad` 加入条件积分抗饱和 |
| `ControlLib/src/pid.cpp` | 同步加入条件积分抗饱和 |

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

### 应用例子

四轮底盘右后轮 CAN 反馈中断，持续 40 ms 后 `motors[3].offline()` 变为 true：

- 该轮速度环每周期被 `clean()`，下发电流置 0；
- 功率分配跳过该轮，只为它保留 `k3/4` 固定损耗；
- 其余三轮继续按逆解结果驱动，机器人仍可移动或平稳停车，但因坏的轮子仍会拖着机器，所以没法做到准确移动

简化后的处理逻辑：

```cpp
if (motors[i].offline()) {
    wheels_pid[i].clean();
    motors[i].give_current = 0;
}
```

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

### 应用例子

以底盘轮速环为例：目标 1 m/s，轮子被卡住，反馈一直为 0，输出很快达到 `max_out = 14000`。

- 旧实现：`Iout` 继续累加，最终顶到 `max_iout = 2000`；障碍解除后积分需要时间退掉，轮子会先冲一下。
- 新实现：输出饱和且误差同向时停止积分，`Iout` 保持 0；障碍解除后直接按当前误差重新调节。

## 验证

- PID：离线数值对比，未饱和场景新旧输出一致；持续堵转 500 拍时新实现积分保持 0，旧实现达到积分上限。
- 整机：尚未在 Linux 下完整构建，也未实车验证；上车前需完整编译并做台架测试。
