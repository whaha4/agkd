# join() 与 detach() 线程控制

## 一、join()：等你跑完

**含义**：当前线程阻塞在 `join()` 这一行，直到目标线程执行完毕才继续。

### 基本用法

```cpp
#include <thread>
#include <iostream>

void worker() {
    std::cout << "工人：干活...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "工人：干完了！\n";
}

int main() {
    std::thread t(worker);  // 创建线程，开始干活
    t.join();               // ← 主线程停在这里，等 worker() 结束
    std::cout << "主线程：OK，我继续\n";
}
```

### 时间线

```
主线程:  create(t) ──→ join() ────────────────→ printf("继续")
                        │                        ↑
                        │  阻塞等待 3 秒           │
                        │                        │
工线程:                └→ worker() ──干活────→ 结束
```

### 在这个项目里

`std::jthread` 的析构函数**自动调用 `join()`**：

```cpp
// C++20 std::jthread 析构
~jthread() {
    if (joinable()) {
        join();  // 自动等待线程结束
    }
}
```

所以 `Robot_ctrl` 里的 `threads` 销毁时，每个 `jthread` 都会 join 对应的模块线程。但因为那些模块线程都是 `while(true)` 死循环，实际永远不会返回。

---

## 二、detach()：切断关系，各管各的

**含义**：解除 jthread 对象和真实线程之间的绑定。线程继续自己跑，jthread 对象可以安全销毁。

### 基本用法

```cpp
void worker() {
    while (true) {
        // 后台任务，永远不需要 join
    }
}

int main() {
    std::thread t(worker);
    t.detach();     // ← "你自己跑，我不管了"
                    //    t 可以安全销毁，不会有 terminate
    // 主线程继续...
}
```

### 图示

```
join():                              detach():

 jthread ──绑定── 线程               jthread  ✂  线程
   │                │                  │          │
   └─ join() → 等你跑完，阻塞           │    自己继续跑
                                      ~jthread  ← 不阻塞
                                      安全销毁
```

### 在这个项目里的使用

**1. IO 管理器析构**（[include/io/io.hpp](GKD_Control/include/io/io.hpp#L22-L25)）

```cpp
~IO() {
    for (auto &handle : io_handles) {
        handle.detach();     // IO 线程是死循环，等不到结束
    }
    io_handles.clear();      // 安全销毁 jthread 对象
}
```

**2. 自瞄超时检测线程**（[src/gimbal/gimbal_temp.cc](GKD_Control/src/gimbal/gimbal_temp.cc#L170)）

```cpp
std::thread check_auto_aim([this] {
    while (true) {
        if (now - receive_auto_aim > 300ms) {
            robot_set->shoot_open &= ~config.gimbal_id;
        }
        UserLib::sleep_ms(10);
    }
});
check_auto_aim.detach();  // 独立后台运行，不需要 join
```

---

## 三、什么都不写：std::terminate() 崩溃

**含义**：`std::thread` 对象析构时，如果线程还在跑且既没 join 也没 detach，C++ 标准要求调用 `std::terminate()` 终止程序。

### 为什么设计成这样？

`std::thread` 析构时面临三种状态：

| 状态 | 安全？ | 原因 |
|---|---|---|
| `joinable() == false` | ✅ | 线程已结束或已被 join/detach |
| `joinable() == true` + 已 join | ✅ | "我等它跑完" |
| `joinable() == true` + 已 detach | ✅ | "我不管了，它自己跑" |
| `joinable() == true` + **什么都没调** | ❌ | 不知道该等还是不管 → 炸 |

设计师的逻辑：**宁可马上炸掉让你发现 bug，也不要悄悄留下失控的线程**（可能访问已销毁的对象 → 更隐蔽的崩溃）。

### 代码演示

```cpp
void worker() {
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

int main() {
    std::thread t(worker);
    // 没写 t.join() 也没写 t.detach()
    return 0;   // ← t 析构 → joinable() == true → std::terminate()
                //   输出: terminate called without an active exception
                //         Aborted (core dumped)
}
```

### `std::terminate()` 是什么？

- 立即终止进程，不弹对话框，不回 main
- 不执行后面的析构函数，不清理栈
- 等价于 Linux 上收到 `SIGABRT`

---

## 四、对比总结

| | join() | detach() | 什么都不写 |
|---|---|---|---|
| 线程结束后 | jthread 安全销毁 | jthread 安全销毁 | — |
| 线程还在跑时 | **阻塞等待**直到结束 | **立即返回**，线程独立跑 | `std::terminate()` 崩溃 |
| 之后能再控制线程吗 | 不能 | 不能（已断开） | 程序已死 |
| 风险 | 死循环导致永久阻塞 | 线程可能访问已销毁对象 | **直接崩** |
| 适用场景 | 需要等待结果 | 后台任务，不需要等 | 永远不应该出现 |

---

## 五、jthread vs thread

| | `std::thread` | `std::jthread` |
|---|---|---|
| 析构时 | 若 joinable → `terminate()` 崩溃 | 若 joinable → 自动 `join()` |
| 是否安全 | 必须手动 join/detach | 自动处理 |
| C++ 版本 | C++11 | C++20 |
| 本项目使用 | `check_auto_aim` 线程 | `Robot_ctrl::threads` |

---

## 六、本项目线程全貌

```
Robot_ctrl 的 threads (std::jthread × N):
  ├── gimbal.task()        ← while(true), join() 永远等不到
  ├── chassis.task()       ← while(true), join() 永远等不到
  └── referee.task()      ← while(true), join() 永远等不到

IO 管理器的 io_handles (std::thread × M):
  ├── Can_interface::task()     ← for(;;), detach() 放生
  ├── Serial_interface::task()  ← while(true), detach() 放生
  └── Socket::task()            ← while(true), detach() 放生

独立的 detach 线程:
  └── check_auto_aim  ← while(true), std::thread + detach()
```

程序实际退出靠 `exit(-1)` 或断电，不依赖析构函数正常返回。
