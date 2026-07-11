# 底盘-only 改动笔记

> 目标：砍掉云台/射击/IMU/自瞄，只保留 4 个底盘麦轮 + 键盘控制

---

## 已完成的改动

### 1. `config_project.hpp`

| 行 | 改动 | 状态 |
|----|------|------|
| 16 | `CanInitList` 删掉 `"CAN_GIMBAL"` | ✅ |
| 18 | `SocketInitList` 改为 `{}` | ✅ |
| 20 | `SerialInitList` 改为 `{}` | ✅ |
| 24 | `rc_controller_serial` 还在 | ⚠️ 后面 robot_controller.cc 需传空字符串 |

### 2. `robot_controller.cc`

| 行 | 改动 | 状态 |
|----|------|------|
| 35 | `gimbal.init(robot_set)` 注释掉 | ✅ |
| 41 | `gimbal.init_task` 线程注释掉 | ✅ |
| 50 | `gimbal.task()` 线程注释掉 | ✅ |

### 3. `chassis.cc`（底盘电流打印）

| 行 | 改动 | 状态 |
|----|------|------|
| 130-142 | 加了 LOG_INFO 打印电流（含 `last = now`） | ✅ |

---

## 还没做的事

1. **键盘控制** —— 还没有加 `keyboard_task` 函数
2. **rc_controller_serial** —— 删掉或 robot_controller.cc 构造传 `""`
3. **砍掉 referee** —— 没有裁判系统的话 `referee.init()` + referee 线程可砍掉（不砍也不崩溃，只是收不到裁判帧打日志而已）
4. **确认 CAN 网卡名** —— `"CAN_CHASSIS"` 必须是 mini PC 上真实的 Socket CAN 网卡名（如 `can0`），否则 `bind()` 直接 `exit(-1)`
   ```bash
   ip link show | grep -i can
   ```
