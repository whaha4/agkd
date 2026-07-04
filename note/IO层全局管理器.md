第 0 层：三个全局 IO 管理器
一切从三个全局变量开始。在 io.hpp：


template<typename T>
IO<T> io;       // ← 声明了一个模板变量

// 在 .cc 文件里（隐式实例化）:
IO<CAN> io<CAN>;          // CAN 设备注册表   (CAN = Can_interface)
IO<SERIAL> io<SERIAL>;    // 串口设备注册表   (SERIAL = Serial_interface)
IO<SOCKET> io<SOCKET>;    // Socket 设备注册表 (SOCKET = Server_socket_interface)
三个全局字典，key 是字符串名字，value 是指向 IO 设备的指针。整个程序任何地方都能通过 IO::io<CAN>[名字] 找到对应的 CAN 通道。

第 1 层：IO 管理器的本质——一个会启动线程的字典
代码很短，直接看完。IO<T> 就是对一个 unordered_map 的包装，加上线程管理：


template<typename T>
class IO {
private:
    std::unordered_map<std::string, T*> data;       // ← "名字 → 设备指针"
    std::vector<std::thread> io_handles;            // ← 所有设备的 task() 线程

public:
    // ① 查找：通过名字找设备
    T* operator[](const std::string &key) const {
        auto p = data.find(key);
        if (p == data.end()) {
            LOG_ERR("no device named %s\n", key.c_str());
            return nullptr;      // 找不到就返回 nullptr
        }
        return p->second;
    }

    // ② 注册：存入 map + 启动线程
    void insert(T &device) {
        data[device.name] = &device;                // 存进字典
        io_handles.emplace_back(
            std::thread([&]() { device.task(); })   // 起一个新线程跑 task()
        );
    }

    // ③ 销毁：放生所有线程
    ~IO() {
        for (auto &handle : io_handles) {
            handle.detach();      // 不和死循环的 task() 较劲
        }
    }
};
你可以把它看成：


io<CAN> 这个全局变量：
┌──────────────────────────────────┐
│ unordered_map<string, Can*>      │
│                                  │
│ "CAN_CHASSIS"  → Can_interface①  │  ← 底盘 CAN 总线
│ "CAN_GIMBAL"   → Can_interface②  │  ← 云台 CAN 总线
│ "CAN_BULLET"   → Can_interface③  │  ← 发射 CAN 总线
│                                  │
│ threads: [task①, task②, task③]  │  ← 三个线程在跑
└──────────────────────────────────┘
insert() 做了什么？


insert("CAN_CHASSIS")
    │
    ├── 1. data["CAN_CHASSIS"] = &那个Can设备
    │      （以后通过 io<CAN>["CAN_CHASSIS"] 就能拿到它）
    │
    └── 2. 启动一个新线程，里面跑 Can_interface::task()
           （这个 task() 死循环读 CAN 帧）
第 2 层：具体的 IO 通道
2.1 CAN：Linux SocketCAN 的包装
只看核心，去掉错误处理：


class Can_interface : public Callback_key<uint32_t, can_frame> {
public:
    std::string name;        // "CAN_CHASSIS" 之类的名字
    int soket_id;            // Linux socket 文件描述符

    // 初始化：打开 CAN 设备
    void init(const char *can_channel) {
        soket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // 创建 CAN socket
        strcpy(ifr->ifr_name, can_channel);             // 设置接口名（如 "can0"）
        ioctl(soket_id, SIOCGIFINDEX, ifr);             // 获取接口索引
        bind(soket_id, ...);                             // 绑定
    }

    // 死循环：不停读 CAN 帧
    bool task() {
        for (;;) {
            read(soket_id, &frame_r, sizeof(can_frame)); // 阻塞读一帧
            callback_key(frame_r.can_id, frame_r);       // 按 CAN ID 分发
        }
    }

    // 发 CAN 帧
    bool send(const can_frame &frame) {
        write(soket_id, &frame, sizeof(can_frame));
    }
};
task() 在干什么？


while(1) {
    read();    // 阻塞在 Linux socket，直到有 CAN 帧到达
               // 比如收到一帧：can_id = 0x201, data = [转子角度, 转速, 电流, 温度]
    
    callback_key(0x201, frame);
        │
        └→ 查表：0x201 注册了谁的回调？
                └→ 调用 motor.unpack(frame)  // 更新电机的角度/转速
}
2.2 Serial：串口的包装

class Serial_interface : public serial::Serial,        // 继承第三方串口库
                         public Callback<ReceivePacket_IMU,  // 按类型分发
                                         ReceivePacket_RC_CTRL> {
public:
    std::string name;         // "/dev/IMU_SMALL_YAW"
    ReceivePacket_IMU imu_pkg;   // 解析出来的 IMU 数据
    ReceivePacket_RC_CTRL rc_pkg; // 解析出来的遥控器数据

    void task() {
        while (true) {
            read(&head0, 1);    // 读一个字节
            if (head0 == 0x55) {                    // 旧协议头
                read(&head1, 1);
                if (head1 == 0xAA) {
                    read(&header, 1);               // 读 ID
                    unpack(header);                  // 解析包体
                }
            } else if (head0 == 0x5A) {             // CH10X 新协议头
                // ... CRC16 校验 + 解析
            }
        }
    }

    void unpack(uint8_t pkg_id) {
        if (pkg_id == 1) {                          // IMU 数据包
            memcpy(buffer, read(...).c_str(), sizeof(IMU));
            callback(imu_pkg);                      // 分发给 IMU 设备
        } else if (pkg_id == 2) {                   // 遥控器数据包
            memcpy(buffer, read(...).c_str(), sizeof(RC));
            callback(rc_pkg);                       // 分发给遥控器设备
        }
    }
};
关键：一条串口线上跑两种数据（IMU + 遥控器），靠 unpack() 里的 pkg_id 区分，然后用 callback() 按类型分发给不同的设备。

2.3 Socket：UDP 的包装

class Server_socket_interface : public Callback<ReceiveNavigationInfo>,
                                public Callback_key<uint8_t, Auto_aim_control> {
public:
    std::string name;          // "AUTO_AIM_CONTROL"
    map<uint8_t, sockaddr_in> clients;  // header → 客户端地址

    void task() {
        while (true) {
            recvfrom(sockfd, buffer, 256, ...);  // 阻塞等待 UDP 包
            uint8_t header = buffer[0];           // 第一个字节是 header
            switch (header) {
                case 0x37:   // 导航数据
                    callback(ReceiveNavigationInfo);
                    break;
                default:     // 自瞄数据（0x6A 等）
                    callback_key(header, Auto_aim_control);
                    break;
            }
        }
    }

    void send(const T &pkg) {
        uint8_t header = *(uint8_t *)(&pkg);              // 取 header 字节
        auto client = clients.find(header)->second;        // 找到对应的客户端地址
        sendto(sockfd, &pkg, sizeof(pkg), ..., client);   // 发给它
    }
};
send() 的巧妙之处：用结构体第一个字节（header）作为 key，自动路由到正确的客户端地址。

第 3 层：回调系统——数据怎么找到"主人"
Callback_key（按键值分发）
用于同一条线上有多种设备，靠 ID 区分的场景（CAN 按 can_id，Socket 按 header）：


template<typename Key, typename ...Args>
class Callback_key {
private:
    std::map<Key, std::function<void(const Args&...)>> callback_map;
    //        ↑                                     ↑
    //      "0x201"                        function(can_frame) { ... }

public:
    // 注册：把 ID 和函数配对
    void register_callback_key(const Key &key, const function &fun) {
        callback_map[key] = fun;
    }

    // 调用：收到数据时查表
    void callback_key(const Key &key, const Args &... args) {
        if (callback_map.count(key)) {
            callback_map[key](args...);  // 找到就调用
        }
    }
};
用起来就是：


// 注册
can->register_callback_key(0x201, [&](can_frame f) {
    motor.unpack(f);     // ID=0x201 的帧归这个电机
});

// 分发（在 task() 里）
can->callback_key(frame.can_id, frame);
// 如果 frame.can_id == 0x201 → 调用上面那个 lambda → motor.unpack()
// 如果 frame.can_id == 0x202 → 调另外的
// 如果 frame.can_id == 0x999 → 没人注册 → 什么都不做
Callback（按类型分发）
用于同一条线上不同包类型，靠 C++ 类型区分的场景（Serial 同时承载 IMU 和 RC）：


// 注册时指定类型
serial->register_callback<ReceivePacket_IMU>([&](auto &pkg) {
    imu.unpack(pkg);     // IMU 类型的包归 IMU 设备
});

serial->register_callback<ReceivePacket_RC_CTRL>([&](auto &pkg) {
    rc.unpack(pkg);      // RC 类型的包归遥控器设备
});

// 分发时：
callback(imu_pkg);    // → 触发所有 ReceivePacket_IMU 类型的回调
callback(rc_pkg);     // → 触发所有 ReceivePacket_RC_CTRL 类型的回调
第 4 层：设备怎么"接上"IO——以 IMU 为例

// 设备
class IMU : public DeviceBase {
    std::string serial_name;   // "/dev/IMU_SMALL_YAW"

    void enable() {
        // ① 通过全局 IO 管理器，用名字找到串口
        auto serial = IO::io<SERIAL>[serial_name];

        // ② 向这个串口注册回调
        serial->register_callback<ReceivePacket_IMU>([&](auto &pkg) {
            this->unpack(pkg);  // ③ 每次串口收到 IMU 数据，自动调到这里
        });
    }

    void unpack(const ReceivePacket_IMU &pkg) {
        yaw   = rad_format(pkg.yaw * PI/180);      // 度 → 弧度
        pitch = rad_format(pkg.pitch * PI/180);
        update_time();  // 标记"我刚收到数据了" → 用于离线检测
    }
};
第 5 层：启动全过程
回到 main.cc 和 robot_controller.cc，看 IO 是怎么起来的：


// ====== 第一步：load_hardware() ======
void Robot_ctrl::load_hardware() {
    // 对配置文件里的每个名字，创建一个 Can_interface 并启动它的 task() 线程
    for (auto& name : {"CAN_CHASSIS", "CAN_BULLET", "CAN_GIMBAL"}) {
        IO::io<CAN>.insert(name);  // → 创建对象 + 存 map + 起线程
    }
    // 同理：创建串口设备
    for (auto& [name, baud, timeout] : SerialInitList) {
        IO::io<SERIAL>.insert(name, baud, timeout);
    }
    // 同理：创建 Socket 设备
    for (auto& name : {"AUTO_AIM_CONTROL"}) {
        IO::io<SOCKET>.insert(name);
    }
}
// 此刻：3条 CAN + 2条 Serial + 1个 Socket 的 task() 线程都在跑了！
// 但还没有人注册回调，收到的数据没人处理。

// ====== 第二步：start_init() ======
void Robot_ctrl::start_init() {
    chassis.init(robot_set);  // 里面：找 CAN → 注册电机回调 → 电机 enable()
    gimbal.init(robot_set);   // 里面：找 Serial → 注册 IMU 回调 → 找 Socket → 注册自瞄回调
    referee.init(robot_set);  // 里面：用自己的串口线程读裁判数据
    // 各模块的 task() 线程启动
}

// ====== 此时 IO 完全就绪 ======
// CAN 上收到电机帧 → callback_key(0x201) → motor.unpack()
// 串口收到 IMU 帧 → callback<IMU>(pkg) → imu.unpack()
// Socket 收到自瞄帧 → callback_key(0x6A, vc) → gimbal 更新目标角度
完整数据流向（一个 CAN 帧的一生）

① 物理层：M3508 电机把自己的角度/转速/温度打包成 CAN 帧发到总线上
        can_id=0x201, data=[AA, BB, CC, DD, EE, FF, 00, 11]

② Linux内核：SocketCAN 驱动收到，放到 socket 缓冲区

③ Can_interface::task()：read(socket) 拿到 can_frame

④ callback_key(0x201, frame)：
    查 map：{0x201: lambda1, 0x202: lambda2, 0x203: lambda3}
    找到 lambda1 → 调用

⑤ lambda1（在 DJIMotorManager::register_motor 里注册的）：
    motor.unpack(frame)
        ├→ data_.rotor_angle = ECD_to_RAD * ecd       // 8192编码 → 弧度
        ├→ data_.rotor_angular_velocity = RPM * 2π/60  // 转速
        └→ update_time()                                // 标记电机在线

⑥ Chassis::task()（500Hz 循环）：
    读到 motor.data_.output_linear_velocity  // ← 第⑤步刚更新的
    用它做 PID 反馈
    算出新电流 → 写 motor.give_current

⑦ DJIMotorManager::task()（1000Hz 循环）：
    收集 4 个电机的 give_current
    打包成一个 can_frame {can_id=0x200, data=[cur1, cur2, cur3, cur4]}
    can->send(frame) → write(socket) → 物理 CAN 总线 → 电机执行
简单说 IO 层就是：三个全局字典 + 每种字典的值是一个死循环读硬件的对象 + 读到了按 ID 或类型叫对应的回调函数。

要我把这个也保存成笔记吗？

IO<CAN> io<CAN>;          // CAN 设备注册表   (CAN = Can_interface)
IO<SERIAL> io<SERIAL>;    // 串口设备注册表   (SERIAL = Serial_interface)
IO<SOCKET> io<SOCKET>;    // Socket 设备注册表 (SOCKET = Server_socket_interface)
这上面语法
这是 C++14 的变量模板（Variable Template）。拆开来看：

第一步：声明模板变量

// io.hpp:57-58
template<typename T>
IO<T> io;        // ← io 是一个"变量模板"，不是具体变量
光这一行，io 还不存在。它只是一个模具——给一个类型 T，就能生成一个对应类型的全局变量。

第二步：实例化（让变量真正诞生）

// 在某个 .cc 文件里：
IO<CAN>    io<CAN>;      // 诞生变量：类型=IO<CAN>,    名字=io<CAN>
IO<SERIAL> io<SERIAL>;   // 诞生变量：类型=IO<SERIAL>, 名字=io<SERIAL>
IO<SOCKET> io<SOCKET>;   // 诞生变量：类型=IO<SOCKET>, 名字=io<SOCKET>
以 IO<CAN> io<CAN>; 为例：


IO<CAN>           io<CAN>;
  ↑                  ↑
  类型                名字
  IO 模板用 CAN      io 模板用 CAN
  实例化后的类型      实例化后的名字
类比：函数模板 vs 变量模板

// 函数模板（你熟悉）
template<typename T>
T max(T a, T b) { return a > b ? a : b; }

max<int>(1, 2);      // 类型=max<int>，调用
max<float>(1.0, 2.0); // 类型=max<float>，调用


// 变量模板（同样道理）
template<typename T>
IO<T> io;             // 模具

IO<CAN> io<CAN>;      // 用 CAN 从模具里造一个变量，类型 IO<CAN>
IO<SERIAL> io<SERIAL>;// 用 SERIAL 从模具里造一个变量，类型 IO<SERIAL>
为什么 <CAN> 出现了两次？
因为在这个项目里，类型和变量用了同一个模板名。换个写法你就懂了：


// 假设变量模板叫 g_io：
template<typename T>
IO<T> g_io;

g_io<CAN>;      // ← 只出现一次 <CAN>，很清晰
g_io<SERIAL>;
但作者把变量模板也命名成了 io，所以变成了：


template<typename T>
IO<T> io;

IO<CAN> io<CAN>;   // 类型 IO<CAN>，变量 io<CAN>，恰好都有 <CAN>
类型和变量是同名模板，实例化时尖括号里的东西一样，看起来像重复了，其实是两个不同的东西。

用在哪里

// 存
IO::io<CAN>.insert("CAN_CHASSIS");   // 给 CAN 注册表插设备

// 取
IO::io<CAN>["CAN_CHASSIS"];          // 用名字从 CAN 注册表取设备
IO::io<SERIAL>["/dev/IMU_SMALL_YAW"];// 用名字从串口注册表取设备
IO::io<SOCKET>["AUTO_AIM_CONTROL"];  // 用名字从 Socket 注册表取设备
三个全局注册表，各管各的，但来自同一个模具。