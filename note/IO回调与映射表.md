# Callback_key 与 map：IO 层的数据分发机制

## 一、Callback_key 是什么

一个**字典类**，里面存了一张 `map<Key, 函数>`。两个核心操作：

```cpp
template<typename Key, typename ...Args>
class Callback_key {
private:
    std::map<Key, std::function<void(const Args&...)>> callback_map;
    //      ↑                                     ↑
    //   键：CAN ID(0x201)                    值：处理函数

public:
    // ① 存入：把 Key 和函数绑定
    void register_callback_key(const Key &key, const function &fun) {
        callback_map[key] = fun;        // 往 map 里塞一对 key→fun
    }

    // ② 取出：按 Key 找到函数并调用
    void callback_key(const Key &key, const Args &... args) {
        if (callback_map.count(key)) {  // key 存在吗？
            callback_map[key](args...); // 存在 → 调函数，传 args 进去
        }
        // 不存在 → 什么都不做
    }
};
```

## 二、Can_interface 怎么继承它

```cpp
// GKD_Control/include/io/can.hpp
class Can_interface : public Callback_key<uint32_t, can_frame>
//                              ↑                 ↑           ↑
//                        模板名=Callback_key    Key=int    Args=can_frame
```

模板实例化后，编译器自动生成：

```cpp
class Can_interface {
private:
    std::map<uint32_t, std::function<void(const can_frame&)>> callback_map;
    //      ↑                                          ↑
    //   0x201, 0x202...                   void(can_frame) 类型的函数

    void register_callback_key(uint32_t key, function<void(const can_frame&)> fn) {
        callback_map[key] = fn;
    }

    void callback_key(uint32_t key, const can_frame &frame) {
        if (callback_map.count(key)) {
            callback_map[key](frame);
        }
    }
};
```

## 三、模板参数 `<uint32_t, can_frame>` 去哪了

```
class Can_interface : public Callback_key< uint32_t , can_frame >
                                             │          │
                                    Key ◄─────┘          │
                                    Args... ◄─────────────┘
                                             │
                                    ┌────────┴────────┐
                                    ▼                 ▼
                            map<Key, function<void(Args...)>>
                            map<uint32_t, function<void(can_frame)>>
                                 ↑                  ↑
                            "哪一帧？"          "帧里有什么？"
```

三个 IO 通道传的类型各不相同：

| IO 通道 | 继承的模板参数 | Key 含义 | Args 含义 |
|---|---|---|---|
| `Can_interface` | `Callback_key<uint32_t, can_frame>` | CAN 帧 ID | 8 字节帧数据 |
| `Server_socket_interface` | `Callback_key<uint8_t, Auto_aim_control>` | 包的 header 字节 | 自瞄控制包 |
| `Serial_interface` | `Callback<IMU包, RC包>` | 不用 key，按**类型**找 | IMU 或遥控器数据 |

## 四、两步走：注册 → 分发

### 注册（电机启动时）

```cpp
// dji_motor.cc —— DJIMotorManager::register_motor()
can_->register_callback_key(
    0x201,                          // key：CAN ID
    [&](const can_frame &frame) {   // value：处理函数
        motor.unpack(frame);        // 解析帧，更新角度/转速
    }
);
```

执行后 map 里多了一行：

```
callback_map:
┌───────┬──────────────────────────────────┐
│ 0x201 │ [lambda] → motor1.unpack(frame)  │  ← 新增的
│ 0x202 │ [lambda] → motor2.unpack(frame)  │
│ ...   │ ...                              │
└───────┴──────────────────────────────────┘
```

### 分发（CAN 帧到达时）

```cpp
// can.cc —— Can_interface::task()
for (;;) {
    read(soket_id, &frame_r, sizeof(can_frame));
    callback_key(frame_r.can_id, frame_r);
    //           ↑ 用 can_id 当 key 去 map 里找
}
```

## 五、完整链路

```
物理 CAN 总线上来一帧：can_id = 0x201
    │
    ▼
Can_interface::task()  read() 拿到 can_frame
    │
    ▼
callback_key(0x201, frame)
    │
    ├→ 开 map：
    │   ┌───────┬──────────────────────┐
    │   │ 0x201 │ motor1.unpack()  ← 命中！✅
    │   │ 0x202 │ motor2.unpack()      │
    │   │ 0x999 │ 没注册 → 默默跳过     │
    │   └───────┴──────────────────────┘
    │
    └→ callback_map[0x201](frame)
        → motor1.unpack(frame)
            ├→ data_.rotor_angle = ...
            ├→ data_.speed_rpm   = ...
            └→ update_time()      // 标记在线
```

## 六、继承关系总图

```
┌─────────────────────────────┐
│      Callback_key           │  ← 父类：提供 map + 查表能力
│   <uint32_t, can_frame>     │
│                             │
│  map<uint32_t, function>    │  ← 这张表在这里
│  register_callback_key()    │     存
│  callback_key()             │     取
└───────────┬─────────────────┘
            │ 继承（public）
┌───────────▼─────────────────┐
│     Can_interface           │  ← 子类：提供 CAN 硬件操作能力
│                             │
│  name, soket_id, addr       │  ← 自己的成员
│  init()    → 打开 CAN 设备   │
│  task()    → 死循环 read()   │
│  send()    → write() 发帧   │
└─────────────────────────────┘
```

`Can_interface` 的职责：**读硬件**。父类 `Callback_key` 的职责：**找到该通知的人**。继承把这两件事拼在一起——一个既能读 CAN，又能按 ID 分发数据的类。
