带我理解总的can是怎么注册并加入字典的
从 load_hardware() 这一行开始，一步一步追到底：


IO::io<CAN>.insert("CAN_CHASSIS")
拆开第一步：io<CAN> 是什么？

// io.hpp:57-58 —— 变量模板声明
template<typename T>
IO<T> io;                // 模具

// 在某个 .cc 里 —— 实例化
IO<CAN> io<CAN>;         // 真实变量：名字 = io<CAN>，类型 = IO<Can_interface>
io<CAN> 是一个全局变量。它的类型是 IO<Can_interface>（因为 CAN = IO::Can_interface）。

它的内部现在是空的：


io<CAN>:
  data    = {}         空字典
  threads = []         空线程列表
拆开第二步：.insert("CAN_CHASSIS") 调的是哪个函数？
insert("CAN_CHASSIS") 传了一个 const char*。IO<T> 类里有两个 insert：


// ① 变参模板版 —— 接受任意参数
template<typename... Args>
void insert(Args &&...args);

// ② 引用版 —— 接受一个已经建好的对象
void insert(T &device);
传了一个参数 "CAN_CHASSIS" → 走 ①。

拆开第三步：变参 insert 做了什么？

// io.hpp:38-40
template<typename... Args>
void insert(Args &&...args) {           // args = "CAN_CHASSIS"
    T *data_p = new T(std::forward<Args>(args)...);
    // T = Can_interface
    // 展开: new Can_interface("CAN_CHASSIS")
    
    insert(*data_p);                     // 调引用版 insert
}
拆开第四步：new Can_interface("CAN_CHASSIS") 做了什么？

// can.cc:10-17
Can_interface::Can_interface(const std::string &name)
    : name(name)                    // ① 把 "CAN_CHASSIS" 存进 this->name
    , Callback_key()                // ② 隐式调用父类默认构造（初始化空 map）
{
    addr = new sockaddr_can;        // ③ 分配地址结构体
    ifr = new ifreq;                // ④ 分配接口请求结构体
    soket_id = -1;                  // ⑤ socket 文件描述符置为 -1
    init_flag = false;
    can_channel_ = name;            // ⑥ 记下通道名
    send_fail_count_ = 0;
    init(name.c_str());             // ⑦ 打开真正的硬件！
}
init("CAN_CHASSIS") 里面：


// can.cc:20-46
soket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);   // Linux 系统调用：创建 CAN socket
strcpy(ifr->ifr_name, "CAN_CHASSIS");            // 指定接口名
ioctl(soket_id, SIOCGIFINDEX, ifr);              // 查接口索引
bind(soket_id, (sockaddr*)addr, sizeof(*addr));  // 绑定 socket 到该接口
init_flag = true;                                 // 标记就绪
此时一个 打开了硬件、可以读写的 Can_interface 对象诞生了：


Can_interface 对象:
  name       = "CAN_CHASSIS"
  soket_id   = 3（某个文件描述符）
  init_flag  = true（硬件就绪）
  callback_map = {}（父类 Callback_key 的空 map，还没人注册回调）
拆开第五步：引用版 insert 做了什么？
拿第 4 步建好的对象，调引用版：


// io.hpp:44-53
void insert(T &device) {                     // device = 刚 new 的对象
    auto &p = data[device.name];             // data["CAN_CHASSIS"]
    if (p != nullptr) {
        throw ...;                           // 重复注册就报错
    }
    p = &device;                             // ① 把对象指针塞进字典
    
    io_handles.emplace_back(                 // ② 创建新线程
        std::thread([&]() { device.task(); }) // 跑 task()
    );
}
执行完的结果

IO::io<CAN>.insert("CAN_CHASSIS") 执行完毕

io<CAN> 全局变量的内部状态：
┌──────────────────────────────────────────────────┐
│ data:                                            │
│   "CAN_CHASSIS" → Can_interface对象              │
│                    ├─ name = "CAN_CHASSIS"        │
│                    ├─ soket_id = 3                │
│                    ├─ init_flag = true            │
│                    └─ callback_map = {} (空)      │
│                                                  │
│ threads:                                         │
│   [线程A] → 正在跑 Can_interface::task()          │
│              for(;;) {                            │
│                  read(soket_id, &frame);          │
│                  callback_key(id, frame);         │
│              }                                   │
└──────────────────────────────────────────────────┘
循环三次后

io<CAN>:
  data = {
    "CAN_CHASSIS" → 对象① (socket=3, task①在读)
    "CAN_GIMBAL"  → 对象② (socket=4, task②在读)
    "CAN_BULLET"  → 对象③ (socket=5, task③在读)
  }
  threads = [task①, task②, task③]
用图总结

IO::io<CAN>.insert("CAN_CHASSIS")
    │
    ├─ ① 进变参 insert("CAN_CHASSIS")
    │      └─ new Can_interface("CAN_CHASSIS")
    │              ├─ name = "CAN_CHASSIS"
    │              ├─ init() → socket() + bind()  打开硬件
    │              └─ 返回对象指针
    │
    ├─ ② 进引用 insert(对象)
    │      ├─ data["CAN_CHASSIS"] = 对象指针      存入字典
    │      └─ 起新线程 → device.task()             开始读 CAN
    │
    └─ ③ 返回
          CAN_CHASSIS 已就绪：硬件打开 + 线程在跑 + 字典可查
字典就是 unordered_map<string, Can_interface*>，key 是名字，value 是对象指针。"注册"就是把指针塞进这个 map，"加入字典"就是 data[name] = &object 这一行