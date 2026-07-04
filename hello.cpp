#include <thread>
#include <iostream>
#include <chrono>

void worker() {
    std::cout << "工人：我在干活...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "工人：干完了\n";
}

int main() {
    std::thread t(worker);   // 创建线程，开始干活
    std::cout << "主线程：我干我的，不等了\n";
    // t.join();             // ← 没写！也没写 t.detach()
    return 0;                // ← 走到这里，t 要析构了！
}
