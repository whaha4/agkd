# 超级电容通信超时降级方案（已回退）

| 项目 | 内容 |
| --- | --- |
| 日期 | 2026-10-26 |
| 状态 | 已从当前代码回退，方案留档 |
| 回退范围 | 3 个文件，+25 / -79 行 |
| 涉及文件 | `robot.hpp`、`super_cap.cc`、`power_controller.cc` |
| 编译验证 | `power_controller.cc`、`super_cap.cc`、`chassis.cc` 通过 MinGW g++ 16.1 `-fsyntax-only` |

## 1. 为什么曾经需要这个方案

0x51 是超级电容唯一的上行状态帧，携带：

- `chassisPower`：电容实测的底盘功率；
- `chassisPowerlimit`：电容反馈的底盘功率上限；
- `capEnergy`：电容剩余能量；
- `errorCode`：故障码（当前代码未使用）。

如果 0x51 因线束、端子、干扰、电容保护等原因中断，`super_cap_info` 会冻结在最后一帧。控制器仍然会拿这些**过期值**做能量环、功率上限和 RLS，无法知道电容真实状态。原方案的目标是：超过 100 ms 收不到 0x51 就切换到保守模式，避免继续按过期数据分配功率。

## 2. 原方案

### 2.1 时间戳与在线判断

`Robot_set` 增加：

```cpp
Types::ReceivePacket_Super_Cap super_cap_info;
// 超级电容状态帧最后一次解析的时间（steady_clock 毫秒），用于通信超时判断
uint32_t super_cap_last_update_ms = 0;
```

`Super_Cap::unpack()` 每收到一帧 0x51：

```cpp
std::memcpy(&robot_set->super_cap_info, frame.data, 8);
update_time();
robot_set->super_cap_last_update_ms = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count());
```

`powerDaemon()` 每 1 ms 轮询：

```cpp
static constexpr uint32_t CAP_FEEDBACK_TIMEOUT_MS = 100;

const uint32_t now_ms = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count());
const bool cap_online =
    robot_set->super_cap_last_update_ms != 0 &&
    (now_ms - robot_set->super_cap_last_update_ms) < CAP_FEEDBACK_TIMEOUT_MS;
```

### 2.2 降级策略

| 项 | 电容在线 | 0x51 超时 |
| --- | --- | --- |
| `estimatedCapEnergy` | `capEnergy / 255 * 2100` | `0` |
| `powerBuff` | `sqrt(capEnergy)` | `0` |
| 能量环目标 | `full=250`、`base=100` | `full=60`、`base=50` |
| `measuredPower` | `super_cap_info.chassisPower` | `estimatedPower`（模型估算） |
| `refereeMaxPower` | `max(chassisPowerlimit, 43)` | `max(裁判 chassis_power_limit, 43)` |
| `powerUpperLimit` | `refereeMaxPower + 300` | `max(refereeMaxPower * 0.85, 40)` |
| 能量环 | 正常执行 | 跳过，`baseMaxPower = fullMaxPower = powerUpperLimit` |
| RLS | 满足条件时更新 | 冻结 |
| 错误标志 | 无 | `CAPDisConnect` |

> 43 W、300 W、0.85、40 W 分别来自 `CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD`、`MAX_CAP_POWER_OUT`、`CAP_REFEREE_BOTH_GG_COE`、`MIN_MAXPOWER_CONFIGURED`。

### 2.3 原代码位置（回退前）

- `power_controller.cc`：`CAP_FEEDBACK_TIMEOUT_MS`、`cap_online`、三处 `if (cap_online)` 分支、`CAPDisConnect`、RLS 条件。
- `super_cap.cc`：`#include <chrono>`、`update_time()`、`super_cap_last_update_ms` 写入。
- `robot.hpp`：`super_cap_last_update_ms` 成员。

## 3. 回退内容

已删除：

- `Robot_set::super_cap_last_update_ms`；
- `CAP_FEEDBACK_TIMEOUT_MS`；
- `Super_Cap::unpack()` 中的 `update_time()` 和时间戳写入；
- `powerDaemon()` 中的 `now_ms`、`cap_online` 和所有在线/离线分支；
- `CAPDisConnect` 错误标志；
- RLS 条件中的 `cap_online`。

已保留（不属于本次回退范围）：

- 0x061 字段错位修复；
- 功率模型/电流指令统一到输出轴量纲；
- 单轮离线处理与三轮运行逻辑；
- `sumError` / `sumPowerRequired` 的除零保护；
- 有轮离线时冻结 RLS（`!any_offline`）；
- `MotorDisconnect` 错误标志。

回退后 `powerDaemon()` 恢复为无条件使用超电反馈：

```cpp
estimatedCapEnergy = robot_set->super_cap_info.capEnergy / 255.0f * 2100.0f;
powerBuff = sqrtf(robot_set->super_cap_info.capEnergy);

fullBuffSet = capFullBuffSet;  // 250
baseBuffSet = capBaseBuffSet;  // 100

refereeMaxPower = fmax(
    robot_set->super_cap_info.chassisPowerlimit,
    CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
powerUpperLimit = refereeMaxPower + MAX_CAP_POWER_OUT;

powerPD_base.set(sqrtf(baseBuffSet));
powerPD_full.set(sqrtf(fullBuffSet));
baseMaxPower = fmax(refereeMaxPower - powerPD_base.out, MIN_MAXPOWER_CONFIGURED);
fullMaxPower = fmax(refereeMaxPower - powerPD_full.out, MIN_MAXPOWER_CONFIGURED);

measuredPower = robot_set->super_cap_info.chassisPower;

if (!any_offline && fabs(measuredPower) > 5.0f) {
    params = rls.update(samples, measuredPower - effectivePower - k3);
    // ...
}
```

## 4. 回退后的当前行为

0x51 中断后，代码**不会检测、不会降级、不会报 `CAPDisConnect`**：

| 项 | 当前行为 |
| --- | --- |
| `capEnergy` / `powerBuff` | 保持最后一帧，能量环继续按旧值运行 |
| `measuredPower` | 保持最后一帧 `chassisPower` |
| `chassisPowerlimit` | 保持最后一帧，`powerUpperLimit` 仍是旧值 + 300 W |
| RLS | 照常更新（仍受 `!any_offline` 和 `|measuredPower| > 5 W` 约束） |
| `CAPDisConnect` | 不会置位 |
| 0x061 | 仍只在 `unpack()` 里每 500 帧发一次；0x51 停则 0x061 也停 |

## 5. 风险

1. **能量环使用过期能量**：最后一帧 `capEnergy` 很高、实际电容已经放空时，能量环仍可能允许较高功率，直到电容自身保护或裁判系统介入。
2. **RLS 标签失真**：`measuredPower` 冻结后仍被当作实测值，`k1/k2` 可能被带偏；之后电容恢复或降级估算时会受影响。
3. **功率上限过期**：`chassisPowerlimit` 冻结后，`powerUpperLimit` 可能高于当前实际允许值。
4. **无故障提示**：上层 UI/状态看不到 `CAPDisConnect`，排查只能靠 CAN 分析仪。
5. **0x061 心跳停止**：这是回退前就存在的问题。`set()` 只在 0x51 回调里调用，0x51 一停，0x061 也停；如果电容固件依赖 0x061 使能/心跳，会影响恢复。

## 6. 如果以后重新实现

### 6.1 最小改法

按第 2 节的方案恢复即可，注意保留当前的：

- 输出轴量纲；
- 单轮离线跳过；
- `!any_offline` 的 RLS 冻结；
- `MotorDisconnect` 标志。

### 6.2 建议改进

1. **0x061 独立发送**：把 `set()` 从 `unpack()` 里拆出来，用独立周期任务发送，避免 0x51 断线导致 0x061 也停。
2. **时间戳用原子类型**：`super_cap_last_update_ms` 由 CAN 接收线程写、功率线程读，当前是普通 `uint32_t`，存在数据竞争；重新实现时用 `std::atomic<uint32_t>`。
3. **恢复加迟滞**：不要一帧恢复就立刻切回在线，可要求连续 N 帧（例如 3 帧）或稳定 20~50 ms，避免临界抖动。
4. **复用 `DeviceBase`**：`Super_Cap` 本身继承 `DeviceBase`，理论上可以 `set_offline_time(100)` + `update_time()` + `offline()`，少一个变量；但当前是私有继承，`offline()` 在 `Super_Cap` 外部不可访问，且 `Power::Manager` 里的 `supercap` 成员未初始化/未使用。若要统一走 `DeviceBase`，需要改成公有继承或加一个 `is_online()` 包装，并解决跨线程访问。原方案因此用了 `Robot_set` 里的时间戳。
5. **明确降级目标**：超时后建议至少做三件事：
   - `measuredPower = estimatedPower`；
   - 功率上限切到裁判系统值并乘保守系数；
   - 冻结 RLS、置 `CAPDisConnect`。
   能量环是否跳过、缓冲目标取多少，需要按实车电容保护策略标定。

### 6.3 测试清单

1. 正常 4 轮、0x51 正常：功率分配与回退前一致。
2. 只断 0x51（保留 0x061）：约 100 ms 后进入降级，`CAPDisConnect` 置位，功率上限切到裁判值 ×0.85。
3. 恢复 0x51：按迟滞条件恢复，RLS 无跳变，能量环无积分反冲。
4. 断 0x51 后继续行驶/小陀螺：确认不会因过期 `capEnergy` 继续请求大功率。
5. 检查 0x061 是否仍在发送（如果做了独立发送改造）。
6. 全目标构建：`make infantry -j8`、`make hero -j8`、`make sentry -j8`；当前仅完成 MinGW 语法检查，未做 Linux 链接和上车验证。

## 7. 回退 diff 摘要

```text
 GKD_Control/include/robot.hpp               |  2 -
 GKD_Control/src/chassis/power_controller.cc | 95 ++++++++---------------------
 GKD_Control/src/device/super_cap.cc         |  7 ---
 3 files changed, 25 insertions(+), 79 deletions(-)
```

当前代码对应位置：

- [power_controller.cc:326](../GKD_Control/src/chassis/power_controller.cc#L326) 能量估计；
- [power_controller.cc:336](../GKD_Control/src/chassis/power_controller.cc#L336) 功率上限；
- [power_controller.cc:346](../GKD_Control/src/chassis/power_controller.cc#L346) 能量环；
- [power_controller.cc:380](../GKD_Control/src/chassis/power_controller.cc#L380) `measuredPower`；
- [power_controller.cc:432](../GKD_Control/src/chassis/power_controller.cc#L432) RLS 条件；
- [super_cap.cc:39](../GKD_Control/src/device/super_cap.cc#L39) 0x51 解析；
- [robot.hpp:66](../GKD_Control/include/robot.hpp#L66) `super_cap_info`。
