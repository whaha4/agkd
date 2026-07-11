# RC Controller 数据流分析

## 数据包结构 (`ReceivePacket_RC_CTRL`)

定义位置：`include/utils/types.hpp:43-58`

```cpp
typedef struct ReceivePacket_RC_CTRL {
    int ch0;      // 摇杆通道0（云台 yaw）
    int ch1;      // 摇杆通道1（云台 pitch）
    int ch2;      // 摇杆通道2（底盘 y 方向）
    int ch3;      // 摇杆通道3（底盘 x 方向）
    int ch4;      // 摇杆通道4（滚轮）
    int s1;       // 拨杆开关1
    int s2;       // 拨杆开关2
    int mouse_x;  // 鼠标 X 位移
    int mouse_y;  // 鼠标 Y 位移
    int mouse_z;  // 鼠标滚轮
    int mouse_l;  // 鼠标左键
    int mouse_r;  // 鼠标右键
    int key;      // 按键位掩码（W/A/S/D/R/F 等）
} __attribute__((packed)) ReceivePacket_RC_CTRL;
```

**关键点**：每一帧数据同时包含摇杆（ch0~ch4）、开关（s1/s2）、鼠标（mouse_*）、按键（key）——全都在一起，不是二选一。

---

## 按键位掩码定义

定义位置：`include/utils/types.hpp:166-172`

```cpp
#define KEY_D     0x1   // 0001
#define KEY_A     0x2   // 0010
#define KEY_W     0x8   // 1000
#define KEY_PRESS 0xff  // 任意键按下
```

---

## 数据接收全链路

```
DJI 遥控器硬件
      │ 串口线
      ▼
Mini PC 串口 (/dev/IMU_HERO)
      │
      ▼
serial_interface.cc:task()        ← 后台线程，逐字节读串口
      │ read(&head0, 1), read(&head1, 1)
      │ 匹配帧头 0x55 0xAA
      ▼
serial_interface.cc:unpack(pkg_id)
      │ pkg_id == 2 → 遥控器包
      │ memcpy(buffer, read(pkg_size).c_str(), pkg_size)
      │ UserLib::fromVector(buffer, &rc_pkg)
      │ callback(rc_pkg)  ← "分发"
      ▼
rc_controller.cc:unpack(ReceivePacket_RC_CTRL &pkg)
      │
      ├─→ 读取 pkg.key  → WASD 按键处理
      ├─→ 读取 pkg.ch0~ch4 → 摇杆处理
      ├─→ 读取 pkg.s1/s2 → 模式切换
      ├─→ 读取 pkg.mouse_* → 云台/射击控制
      └─→ 写入 robot_set → 最终控制底盘/云台/射击
```

---

## WASD vs 摇杆 区分机制

核心在 `rc_controller.cc:129-137`：

```cpp
static bool use_key = false;       // 默认 false = 摇杆模式

if (pkg.key & KEY_PRESS) {        // 遥控器上任一键按下
    use_key = true;               // 永久切到键盘模式
}

if (use_key) {
    return;   // 键盘模式下直接返回，跳过摇杆代码
}
```

逻辑：
1. 默认 `use_key = false`，走摇杆通道（`pkg.ch2/ch3`），比例控制
2. 一旦遥控器上任何键被按下 → `use_key` 永久设为 `true`
3. `use_key == true` 后，只能走键盘通道（`pkg.key`），定速控制
4. 摇杆数据本身一直在发送，只是代码忽略它

**两个字段同时在包里，代码用标志位选择读哪个。**

---

## 各字段控制对应表

| 字段 | 控制对象 | 效果 |
|------|---------|------|
| `key` (W/A/S/D) | 底盘移动 | 定速（+1/0/-1 × speed） |
| `key` (R) | 底盘自旋 | 切换开关 |
| `key` (F) | 摩擦轮 | 开关切换 |
| `ch0` | 云台 yaw | 比例控制 |
| `ch1` | 云台 pitch | 比例控制 |
| `ch2` | 底盘 vy | 比例控制 |
| `ch3` | 底盘 vx | 比例控制 |
| `s1/s2` | 模式切换 | 自瞄/摩擦轮/哨兵跟随 |
| `mouse_x/y` | 云台微调 | 位移累加 |
| `mouse_l/r` | 射击 | 左键/右键射击 |

---

## 注意

- WASD 键盘模式仅在 `#ifndef CONFIG_SENTRY` 内有效（哨兵用不了）
- 哨兵的控制逻辑在 `#ifdef CONFIG_SENTRY` 分支里，主要靠 s2 开关 + ch4 滚轮

---

## `use_key == true` 时的行为

```cpp
// ─── 以下永远执行（不受 use_key 影响）───
// 50-68行：  WASD → vx_set, vy_set（定速，但 key=0 时就是 0）
// 75-84行：  R 键 → 切换自旋
// 87-97行：  F 键 → 切换摩擦轮

// ─── 135行：键盘模式检查，提前 return ───
if (use_key) {
    return;   // 下面全部跳过！
}

// ─── 以下只有 use_key == false 才执行 ───
// 101-105行：鼠标右键/s1 → 自瞄开关
// 108-112行：鼠标左键/ch4 → 射击控制
// 117-122行：鼠标位移 → 云台微调
// 144-146行：摇杆 ch3 → 底盘 vx（比例控制）
// 147行：    摇杆 ch2 → 底盘 vy（比例控制）
// 150行：    摇杆 ch0 → 云台 yaw
// 153行：    摇杆 ch1 → 云台 pitch
// 159-166行：s1/s2 → 自旋/摩擦轮/哨兵跟随
```

| 功能 | keyboard 模式能用？ | 原因 |
|------|-------------------|------|
| WASD 移动 | ✅ | 在 `return` 之前 |
| R 键自旋 | ✅ | 在 `return` 之前 |
| F 键摩擦轮 | ✅ | 在 `return` 之前 |
| 鼠标自瞄 | ❌ | 被 `return` 跳过 |
| 鼠标射击 | ❌ | 被 `return` 跳过 |
| 云台微调 | ❌ | 被 `return` 跳过 |
| 摇杆比例速度 | ❌ | 被 `return` 跳过 |
| s1/s2 模式切换 | ❌ | 被 `return` 跳过 |

**注意**：`use_key` 是 `static` 变量，一旦变为 `true`，没有任何代码把它设回 `false`。
意味着只要遥控器上按过一次键，摇杆就永久失效，只能重启程序恢复。

---

## 手柄数据处理的代码位置

都在 `rc_controller.cc` 的 `unpack()` 函数内：

| 行号 | 代码 | 功能 |
|------|------|------|
| 101-106 | `pkg.mouse_r \|\| pkg.s1` | 鼠标右键/s1 → 自瞄开 |
| 108-112 | `pkg.mouse_l \|\| pkg.ch4` | 鼠标左键/滚轮 → 射击 |
| 117-122 | `pkg.mouse_x / 10000.` | 鼠标位移 → 云台 yaw/pitch 微调 |
| 144 | `pkg.ch3 / RC_SCALE * SPEED` | 摇杆 ch3 → 底盘 vx（前后） |
| 147 | `pkg.ch2 / RC_SCALE * SPEED` | 摇杆 ch2 → 底盘 vy（左右） |
| 150 | `pkg.ch0 / RC_SCALE * SENS` | 摇杆 ch0 → 云台 yaw |
| 153 | `pkg.ch1 / RC_SCALE * SENS` | 摇杆 ch1 → 云台 pitch |
| 159-160 | `pkg.s1 == S1_UP` | s1 开关 → 自旋 |
| 165 | `pkg.s2 == S2_UP` | s2 开关 → 摩擦轮 |
| 169-181 | `#ifdef CONFIG_SENTRY` | 哨兵模式：s2+ch4 → 跟随/射击 |
