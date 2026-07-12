# 底盘-only 改动笔记

> 目标：砍掉云台/射击/IMU/自瞄，只保留 4 个底盘麦轮 + 键盘控制（UDP）

---

## 已完成的改动

### 1. `config_project.hpp`

| 行 | 改动 | 状态 |
|----|------|------|
| 16 | `CanInitList` 删掉 `"CAN_GIMBAL"`，只留 `"CAN_CHASSIS"` | ✅ |
| 18 | `SocketInitList` = `{"KEYBOARD_CONTROL"}`，为 UDP 键盘控制做准备 | ✅ |
| 20-22 | `SerialInitList` = `{}`，砍掉所有串口 | ✅ |

### 2. `robot_controller.cc`

| 行 | 改动 | 状态 |
|----|------|------|
| 28 | `referee.init(robot_set)` 注释掉 | ✅ |
| 35 | `gimbal.init(robot_set)` 注释掉 | ✅ |
| 41 | `gimbal.init_task` 线程注释掉 | ✅ |
| 50 | `gimbal.task()` 线程注释掉 | ✅ |
| 52 | `set_mode(ROBOT_FOLLOW_GIMBAL)` 退出无力模式 | ✅ |
| 54-55 | referee 两条线程注释掉 | ✅ |

### 3. `chassis.cc`

| 行 | 改动 | 状态 |
|----|------|------|
| 49-50 | 裁判断电检查改为 `if(false)`，跳过 | ✅ |
| 130-142 | 加了 LOG_INFO 打印电流 | ✅ |

---

## 可编译，还没做的事

1. **键盘控制 Python 脚本** —— 还没有写
2. **robot_controller.cc 注册键盘回调** —— Socket 口已建（`KEYBOARD_CONTROL`），回调还没注册
3. **确认 CAN 网卡名** —— `"CAN_CHASSIS"` 必须是 mini PC 上真实的 Socket CAN 网卡名（如 `can0`）
   ```bash
   ip link show | grep -i can
   ```
